// ============================================================================
// Move — Slave node (RP2350)
//
// Per agent.md topology:
//   Hub(Move) ↔ Move：PIO UART (SM0), 921600 baud, COBS framing
//
// Core allocation (agent.md §核心分配):
//   Core 0 (loop):   PIO UART RX/TX → COBS → CRC → notify Core 1
//   Core 1 (loop1):  Triggered by Core 0 → execute mecanum wheel motor control
//
// Core 1 sync pattern (agent.md §核心間同步機制):
//   Core 0 writes SlaveCommand to lock-free double-buffer with release store
//   Core 1 spinloops with acquire load, memcpy snapshot, executes, loops back
// ============================================================================
#include "signal_shared.h"
#include <Arduino.h>
#include <atomic>
#include <hardware/uart.h>

// ============================================================================
// Core 0 ↔ Core 1 lock-free command buffer (agent.md §核心間通訊)
// ============================================================================
static struct SlaveCommand {
    ChassisSnapshot snapshot;
    bool            estopActive;
} cmdBuf[2];
static std::atomic<uint8_t> cmdWriteIdx{0};
static std::atomic<bool>    estopActive{false};

// ============================================================================
// PIO UART state (placeholder — PIO SM0 will be configured here)
// Per agent.md: Slaves use PIO UART, not hardware UART peripherals.
// PIO SM0 handles both TX and RX at 921600 baud.
// PIO program loaded from .pio file; SM configured with proper GPIOs.
// ============================================================================
static constexpr uint8_t PIO_SLAVE_TX_PIN = 4;  // GP4 (TBD — per hub ↔ slave pinout)
static constexpr uint8_t PIO_SLAVE_RX_PIN = 5;  // GP5 (TBD)

// COBS scan state — .bss allocated, no heap
static uint8_t parseWorkBuf[512];
static size_t  workBufIdx = 0;
static uint8_t lastFrameSeq  = 0;
static bool    firstFrame    = true;

// E-Stop timeout tracking
static uint64_t lastValidCmdTime   = 0;
static bool     motorPowerOn       = false;

// Heartbeat
static uint64_t lastHeartbeatTime  = 0;

// ============================================================================
// Forward declarations for Core 1 PIO motor control
// Stubs — replace with real MotorControl library once available.
// ============================================================================
namespace MotorControl {
void init() { /* TODO: init PIO PWM / encoder */ }
void executeMotion(const ChassisSnapshot&) { /* TODO: apply motor setpoints */ }
void estop() { /* TODO: zero all PWM */ }
} // namespace MotorControl

// ============================================================================
// Core 0: Process a fully decoded + CRC-verified COBS frame
//   - FrameSeq anti-duplicate
//   - Write to Core 1 double-buffer with release semantics
//   - Track lastValidCmdTime for E-Stop timeout
// ============================================================================
static void processCommand(const ControlPacket& pkt) {
    // FrameSeq duplicate check
    if (!firstFrame && pkt.frameSeq == lastFrameSeq)
        return;
    firstFrame   = false;
    lastFrameSeq = pkt.frameSeq;

    // Translate wire-format ControlPacket to in-memory ChassisSnapshot
    ChassisSnapshot snap;
    snap.vx               = static_cast<float>(pkt.vx);
    snap.vy               = static_cast<float>(pkt.vy);
    snap.omega            = static_cast<float>(pkt.omega);
    snap.mode             = pkt.mode;
    snap.frameSeq         = pkt.frameSeq;
    snap.lastArrivalTick  = time_us_64();

    // Write to double-buffer with release semantics (agent.md §核心間通訊)
    uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
    cmdBuf[idx].snapshot    = snap;
    cmdBuf[idx].estopActive = false;
    cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

    lastValidCmdTime = snap.lastArrivalTick;
    motorPowerOn     = true;
}

// ============================================================================
// Core 0: Process E-Stop from Hub (targetID == BROADCAST, msgType == ESTOP)
// ============================================================================
static void processEStop(const ControlPacket& pkt) {
    (void)pkt;
    estopActive.store(true, std::memory_order_release);

    // Also write to double-buffer so Core 1 immediately sees it
    uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
    cmdBuf[idx].estopActive = true;
    cmdBuf[idx].snapshot.vx = cmdBuf[idx].snapshot.vy =
        cmdBuf[idx].snapshot.omega = 0;
    cmdBuf[idx].snapshot.mode = RobotMode::FAILSAFE;
    cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

    motorPowerOn = false;
}

// ============================================================================
// Core 0: PIO UART receive wrapper (placeholder until PIO SM0 is connected)
// Non-blocking: returns number of bytes read (max 64), 0 if none available.
// ============================================================================
static size_t pioUartReadBytes(uint8_t* buf, size_t maxLen) {
    // TODO: Replace with actual PIO SM0 RX read
    //   sm = pio_get_index(pio0), SM0
    //   if (pio_sm_get_rx_fifo_level(pio, sm) > 0)
    //       buf[n++] = pio_sm_get(pio, sm) & 0xFF;
    // Stub: no bytes available
    (void)buf;
    (void)maxLen;
    return 0;
}

// ============================================================================
// Core 0: PIO UART send wrapper (placeholder)
// ============================================================================
static void pioUartSendBytes(const uint8_t* buf, size_t len) {
    // TODO: Replace with actual PIO SM0 TX push
    //   for (size_t i = 0; i < len; i++)
    //       pio_sm_put_blocking(pio0, 0, buf[i]);
    (void)buf;
    (void)len;
}

// ============================================================================
// Core 0: COBS scan pass — called each loop() iteration
// Processes up to 64 bytes looking for 0x00 delimiters.
// ============================================================================
static void processPioUartRx() {
    uint8_t rawBuf[64];
    size_t  nRead = pioUartReadBytes(rawBuf, sizeof(rawBuf));
    if (nRead == 0) return;

    for (size_t i = 0; i < nRead; i++) {
        uint8_t byte = rawBuf[i];

        if (byte == 0x00) {
            // End of COBS frame
            if (workBufIdx >= sizeof(ControlPacket)) {
                uint8_t decoded[512];
                size_t  decLen = SignalUtils::cobsDecode(parseWorkBuf,
                                                          workBufIdx,
                                                          decoded);
                if (decLen >= sizeof(ControlPacket)) {
                    const ControlPacket& pkt =
                        *reinterpret_cast<const ControlPacket*>(decoded);

                    // CRC verify
                    uint16_t storedCrc = pkt.crc16;
                    uint16_t calcCrc   = SignalUtils::calculateCrc(
                                             decoded,
                                             sizeof(ControlPacket) - 2);
                    if (storedCrc == calcCrc) {
                        if (pkt.sync == 0xA5) {
                            if (pkt.msgType == MsgType::HEARTBEAT ||
                                pkt.msgType == MsgType::CONTROL) {
                                processCommand(pkt);
                            } else if (pkt.msgType == MsgType::ESTOP) {
                                processEStop(pkt);
                            }
                        }
                    }
                    // CRC mismatch → silently drop (agent.md rule)
                }
            }
            workBufIdx = 0;
        } else {
            if (workBufIdx < sizeof(parseWorkBuf) - 1)
                parseWorkBuf[workBufIdx++] = byte;
        }
    }
}

// ============================================================================
// Core 0: Send heartbeat upstream to Hub via PIO UART
// Heartbeat: 30ms interval, mode=FAILSAFE (agent.md §心跳)
// ============================================================================
static void sendHeartbeat() {
    uint64_t now = time_us_64();
    if (now - lastHeartbeatTime < RobotConfig::HEARTBEAT_INTERVAL_US)
        return;
    lastHeartbeatTime = now;

    static uint8_t hbFrameSeq = 0;

    ControlPacket pkt;
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::HEARTBEAT;
    pkt.frameSeq   = hbFrameSeq++;
    pkt.targetID   = TargetID::HUB;
    pkt.payloadLen = sizeof(ControlPacket) - 6;
    pkt.vx         = 0;
    pkt.vy         = 0;
    pkt.omega      = 0;
    pkt.mode       = RobotMode::FAILSAFE;
    pkt.timestamp_hi = static_cast<uint32_t>(now >> 32);
    pkt.timestamp_lo = static_cast<uint32_t>(now & 0xFFFFFFFF);
    pkt.crc16      = 0;

    pkt.crc16 = SignalUtils::calculateCrc(
                    reinterpret_cast<uint8_t*>(&pkt),
                    sizeof(ControlPacket) - 2);

    uint8_t  cobsBuf[sizeof(ControlPacket) + 2];
    size_t   cobsLen = SignalUtils::cobsEncode(
                          reinterpret_cast<const uint8_t*>(&pkt),
                          sizeof(ControlPacket),
                          cobsBuf);
    cobsBuf[cobsLen] = 0x00;

    pioUartSendBytes(cobsBuf, cobsLen + 1);
}

// ============================================================================
// Core 0: E-Stop timeout checker (agent.md §指令超時保護)
// If no valid command received for >100ms, trigger E-Stop.
// ============================================================================
static void checkTimeout() {
    if (!motorPowerOn) return;

    uint64_t now = time_us_64();
    if (now - lastValidCmdTime > RobotConfig::COMMAND_TIMEOUT_US) {
        // Timeout — notify Core 1
        estopActive.store(true, std::memory_order_release);

        uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
        cmdBuf[idx].estopActive = true;
        cmdBuf[idx].snapshot.vx = cmdBuf[idx].snapshot.vy =
            cmdBuf[idx].snapshot.omega = 0;
        cmdBuf[idx].snapshot.mode = RobotMode::FAILSAFE;
        cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

        motorPowerOn = false;
    }
}

// ============================================================================
// Core 0: setup()
// ============================================================================
void setup() {
    // CRC table is compile-time const — no init needed.

    // TODO: Initialize PIO SM0 for UART TX/RX
    //   pio_sm_claim(pio0, 0);
    //   pio_add_program(pio0, &cobs_uart_program);
    //   cobs_uart_program_init(pio0, 0, PIO_SLAVE_TX_PIN, PIO_SLAVE_RX_PIN, 921600);

    // Initialize motor control on Core 1
    // MotorControl::init() called from setup1() instead

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

// ============================================================================
// Core 0: loop() — COBS frame parsing + UART heartbeat + timeout check
// ============================================================================
void loop() {
    processPioUartRx();
    sendHeartbeat();
    checkTimeout();

    // Keep loop rate high — Core 0 should never block
    delayMicroseconds(20);
}

// ============================================================================
// Core 1: setup1() — Initialize PIO PWM and encoder peripherals
// ============================================================================
void setup1() {
    MotorControl::init();

    // Wait for first command from Core 0 before enabling motors
    motorPowerOn = false;
}

// ============================================================================
// Core 1: loop1() — Spinloop waiting for Core 0 commands
//
// Per agent.md §核心間同步機制:
//   1. acquire-load cmdWriteIdx
//   2. memcpy snapshot from other buffer
//   3. execute motor control
//   4. check E-Stop on each cycle
//   5. loop back
// ============================================================================
void loop1() {
    while (true) {
        // Acquire-load the latest write index (agent.md §核心間通訊)
        uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
        uint8_t readIdx = idx ^ 1;

        // Snapshot command via memcpy (avoids data race on struct fields)
        ChassisSnapshot cmd;
        bool estop;
        memcpy(&cmd, &cmdBuf[readIdx].snapshot, sizeof(cmd));
        estop = cmdBuf[readIdx].estopActive;

        // Check global E-Stop flag too
        if (estop || estopActive.load(std::memory_order_acquire)) {
            MotorControl::estop();
            motorPowerOn = false;
        } else {
            motorPowerOn = true;
            MotorControl::executeMotion(cmd);
        }

        // Tight spin — no delay()
    }
}
