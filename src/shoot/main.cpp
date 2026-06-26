// ============================================================================
// Shoot — Slave 節點 (RP2350)
//
// 根據 agent.md 拓撲：
//   Hub(Shoot) ↔ Shoot：PIO UART (SM1), 921600 baud, COBS 封幀
//
// 核心分配（agent.md §核心分配）：
//   Core 0 (loop)：   PIO UART 收發 → COBS → CRC → 通知 Core 1
//   Core 1 (loop1)：  由 Core 0 觸發 → 執行 BLDC / 步進馬達控制
//
// Core 1 同步模式（agent.md §核心間同步機制）：
//   Core 0 以 release store 寫入 SlaveCommand 至無鎖雙緩衝區
//   Core 1 以 acquire load 自旋等待，memcpy 快照，執行，迴圈
// ============================================================================
#include "signal_shared.h"
#include <Arduino.h>
#include <atomic>
#include <hardware/uart.h>

// ============================================================================
// Core 0 ↔ Core 1 無鎖命令緩衝區（agent.md §核心間通訊）
// ============================================================================
static struct SlaveCommand {
    ChassisSnapshot snapshot;
    bool            estopActive;
} cmdBuf[2];
static std::atomic<uint8_t> cmdWriteIdx{0};
static std::atomic<bool>    estopActive{false};

// ============================================================================
// PIO UART 狀態（佔位 — PIO SM1 將在此處配置）
// 根據 agent.md：Slaves 使用 PIO UART，非硬體 UART 週邊。
// ============================================================================
static constexpr uint8_t PIO_SLAVE_TX_PIN = 16; // GP16 (TBD — per hub ↔ slave pinout)
static constexpr uint8_t PIO_SLAVE_RX_PIN = 17; // GP17 (TBD)

// COBS 掃描狀態 — .bss 分配，無堆疊使用
static uint8_t parseWorkBuf[512];
static size_t  workBufIdx    = 0;
static uint8_t lastFrameSeq  = 0;
static bool    firstFrame    = true;

// E-Stop 逾時追蹤
static uint64_t lastValidCmdTime = 0;
static bool     motorPowerOn     = false;

// 心跳
static uint64_t lastHeartbeatTime = 0;

// ============================================================================
// 前向宣告 — Core 1 PIO 馬達控制
// 樁模組 — 待取得實際馬達函式庫後替換。
// ============================================================================
namespace MotorControl {
void init() { /* TODO: init PIO PWM / encoder */ }
void executeMotion(const ChassisSnapshot&) { /* TODO: apply motor setpoints */ }
void estop() { /* TODO: zero all PWM */ }
} // namespace MotorControl

// ============================================================================
// Core 0：處理已完整解碼且通過 CRC 驗證的 COBS 幀
// ============================================================================
static void processCommand(const ControlPacket& pkt) {
    if (!firstFrame && pkt.frameSeq == lastFrameSeq)
        return;
    firstFrame   = false;
    lastFrameSeq = pkt.frameSeq;

    ChassisSnapshot snap;
    snap.vx              = static_cast<float>(pkt.vx);
    snap.vy              = static_cast<float>(pkt.vy);
    snap.omega           = static_cast<float>(pkt.omega);
    snap.mode            = pkt.mode;
    snap.frameSeq        = pkt.frameSeq;
    snap.lastArrivalTick = time_us_64();

    uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
    cmdBuf[idx].snapshot    = snap;
    cmdBuf[idx].estopActive = false;
    cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

    lastValidCmdTime = snap.lastArrivalTick;
    motorPowerOn     = true;
}

// ============================================================================
// Core 0：處理來自 Hub 的 E-Stop
// ============================================================================
static void processEStop(const ControlPacket& pkt) {
    (void)pkt;
    estopActive.store(true, std::memory_order_release);

    uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
    cmdBuf[idx].estopActive = true;
    cmdBuf[idx].snapshot.vx = cmdBuf[idx].snapshot.vy =
        cmdBuf[idx].snapshot.omega = 0;
    cmdBuf[idx].snapshot.mode = RobotMode::FAILSAFE;
    cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

    motorPowerOn = false;
}

// ============================================================================
// Core 0：PIO UART 接收包裝函式（佔位）
// ============================================================================
static size_t pioUartReadBytes(uint8_t* buf, size_t maxLen) {
    (void)buf;
    (void)maxLen;
    return 0;
}

// ============================================================================
// Core 0：PIO UART 發送包裝函式（佔位）
// ============================================================================
static void pioUartSendBytes(const uint8_t* buf, size_t len) {
    (void)buf;
    (void)len;
}

// ============================================================================
// Core 0：COBS 掃描 — 每次 loop() 迭代
// ============================================================================
static void processPioUartRx() {
    uint8_t rawBuf[64];
    size_t  nRead = pioUartReadBytes(rawBuf, sizeof(rawBuf));
    if (nRead == 0) return;

    for (size_t i = 0; i < nRead; i++) {
        uint8_t byte = rawBuf[i];

        if (byte == 0x00) {
            if (workBufIdx >= sizeof(ControlPacket)) {
                uint8_t decoded[512];
                size_t  decLen = SignalUtils::cobsDecode(parseWorkBuf,
                                                          workBufIdx,
                                                          decoded);
                if (decLen >= sizeof(ControlPacket)) {
                    const ControlPacket& pkt =
                        *reinterpret_cast<const ControlPacket*>(decoded);
                    uint16_t storedCrc = pkt.crc16;
                    uint16_t calcCrc   = SignalUtils::calculateCrc(
                                             decoded,
                                             sizeof(ControlPacket) - 2);
                    if (storedCrc == calcCrc && pkt.sync == 0xA5) {
                        if (pkt.msgType == MsgType::HEARTBEAT ||
                            pkt.msgType == MsgType::CONTROL)
                            processCommand(pkt);
                        else if (pkt.msgType == MsgType::ESTOP)
                            processEStop(pkt);
                    }
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
// Core 0：向上游 Hub 發送心跳
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
// Core 0：逾時檢查（agent.md §指令超時保護）
// ============================================================================
static void checkTimeout() {
    if (!motorPowerOn) return;

    uint64_t now = time_us_64();
    if (now - lastValidCmdTime > RobotConfig::COMMAND_TIMEOUT_US) {
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
// Core 0：setup()
// ============================================================================
void setup() {
    // CRC 表為編譯期常數 — 無需初始化。

    // TODO：初始化 PIO SM1 作為 UART 收發
    //   pio_sm_claim(pio0, 1);
    //   pio_add_program(pio0, &cobs_uart_program);
    //   cobs_uart_program_init(pio0, 1, PIO_SLAVE_TX_PIN, PIO_SLAVE_RX_PIN, 921600);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

// ============================================================================
// Core 0：loop()
// ============================================================================
void loop() {
    processPioUartRx();
    sendHeartbeat();
    checkTimeout();
    delayMicroseconds(20);
}

// ============================================================================
// Core 1：setup1()
// ============================================================================
void setup1() {
    MotorControl::init();
    motorPowerOn = false;
}

// ============================================================================
// Core 1：loop1() — 自旋等待 Core 0 的命令
// ============================================================================
void loop1() {
    while (true) {
        uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
        uint8_t readIdx = idx ^ 1;

        ChassisSnapshot cmd;
        bool estop;
        memcpy(&cmd, &cmdBuf[readIdx].snapshot, sizeof(cmd));
        estop = cmdBuf[readIdx].estopActive;

        if (estop || estopActive.load(std::memory_order_acquire)) {
            MotorControl::estop();
            motorPowerOn = false;
        } else {
            motorPowerOn = true;
            MotorControl::executeMotion(cmd);
        }
    }
}
