// ============================================================================
// Shoot — Slave 節點 (RP2350)
//
// 根據 agent.md 拓撲：
//   Hub(PIO SM1) ↔ Shoot：硬體 UART0, 921600 baud, COBS 封幀
//
// 核心分配（agent.md §核心分配）：
//   Core 0 (loop)：   硬體 UART0 收發 → COBS → CRC → 通知 Core 1
//   Core 1 (loop1)：  由 Core 0 觸發 → 執行 BLDC / 步進馬達控制
//
// Core 1 同步模式（agent.md §核心間同步機制）：
//   Core 0 以 release store 寫入 SlaveCommand 至無鎖雙緩衝區
//   Core 1 以 acquire load 自旋等待，memcpy 快照，執行，迴圈
//
// UART 接線：
//   Hub PIO SM1 TX(GP8) ──── Shoot UART0 RX(GP1)
//   Hub PIO SM1 RX(GP9) ──── Shoot UART0 TX(GP0)
// ============================================================================
#include "ChassisComms.h"
#include "signal_shared.h"
#include <Arduino.h>
#include <atomic>
#include <cstring>

// ============================================================================
// Core 0 ↔ Core 1 無鎖命令緩衝區（agent.md §核心間通訊）
// ============================================================================
static struct SlaveCommand {
    ChassisSnapshot snapshot;
    bool            estopActive;
} cmdBuf[2];
static std::atomic<uint8_t> cmdWriteIdx{0};
static std::atomic<bool>    estopActive{false};

// E-Stop 逾時追蹤
static uint64_t lastValidCmdTime = 0;
static bool     motorPowerOn     = false;

// ============================================================================
// 硬體 UART0 + ChassisComms — 與 Hub 通訊
// ============================================================================
// Serial1 = UART0 on RP2350（Earlephilhower mapping）
// 預設腳位 TX=GP0, RX=GP1
// ChassisComms 綁定此 UART，心跳 30ms 回送 Hub
static ChassisComms hubComms(&Serial1, TargetID::HUB, RobotConfig::HEARTBEAT_INTERVAL_US);

// ============================================================================
// 前向宣告 — Core 1 馬達控制
// ============================================================================
namespace MotorControl {
void init() { /* TODO: init PIO PWM / encoder / BLDC */ }
void executeMotion(const ChassisSnapshot&) { /* TODO: apply motor setpoints */ }
void estop() { /* TODO: zero all PWM */ }
} // namespace MotorControl

// ============================================================================
// Core 0：收到 Hub 控制幀的回呼（由 ChassisComms 觸發）
// ============================================================================
static void onHubFrame(const ControlPacket& pkt) {
    // E-Stop 廣播處理（agent.md §E-Stop 安全機制）
    if(pkt.msgType == MsgType::ESTOP || pkt.targetID == TargetID::BROADCAST) {
        estopActive.store(true, std::memory_order_release);

        uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
        cmdBuf[idx].estopActive = true;
        cmdBuf[idx].snapshot.vx = cmdBuf[idx].snapshot.vy =
            cmdBuf[idx].snapshot.omega = 0;
        cmdBuf[idx].snapshot.mode = RobotMode::FAILSAFE;
        cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

        motorPowerOn = false;
        return;
    }

    // Bypass 非控制幀
    if(pkt.msgType != MsgType::CONTROL) {
        return;
    }

    // 過濾非自己的幀
    if(pkt.targetID != TargetID::SHOOT && pkt.targetID != TargetID::BROADCAST) {
        return;
    }

    // 更新最後有效幀時間
    lastValidCmdTime = time_us_64();

    // 解碼命令，寫入雙緩衝區
    ChassisSnapshot snap;
    constexpr float SCALE = RobotConfig::MAX_RPS / 127.0f;
    snap.vx              = static_cast<float>(pkt.vx) * SCALE;
    snap.vy              = static_cast<float>(pkt.vy) * SCALE;
    snap.omega           = static_cast<float>(pkt.omega) * SCALE;
    snap.mode            = pkt.mode;
    snap.frameSeq        = pkt.frameSeq;
    snap.lastArrivalTick = time_us_64();

    uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
    cmdBuf[idx].snapshot    = snap;
    cmdBuf[idx].estopActive = false;
    cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

    // 如果 E-Stop 曾觸發，現在收到有效幀 → 解除
    if(estopActive.load(std::memory_order_acquire)) {
        estopActive.store(false, std::memory_order_release);
    }
    motorPowerOn = true;
}

// ============================================================================
// Core 0：setup()
// ============================================================================
void setup() {
    // 硬體 UART0 — 921600 baud，預設腳位 TX=GP0, RX=GP1
    Serial1.begin(RobotConfig::PIO_BAUDRATE);
    hubComms.begin();
    hubComms.setFrameCallback(onHubFrame);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

// ============================================================================
// Core 0：loop()
// ============================================================================
void loop() {
    // 1. ChassisComms 驅動收發：UART 讀取 → COBS 解碼 → CRC → onHubFrame
    hubComms.update();

    // 2. 逾時檢查（agent.md §指令超時保護）
    if(motorPowerOn &&
       (time_us_64() - lastValidCmdTime > RobotConfig::COMMAND_TIMEOUT_US)) {
        estopActive.store(true, std::memory_order_release);

        uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
        cmdBuf[idx].estopActive = true;
        cmdBuf[idx].snapshot.vx = cmdBuf[idx].snapshot.vy =
            cmdBuf[idx].snapshot.omega = 0;
        cmdBuf[idx].snapshot.mode = RobotMode::FAILSAFE;
        cmdWriteIdx.store(idx ^ 1, std::memory_order_release);

        motorPowerOn = false;
    }

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
    while(true) {
        uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire);
        uint8_t readIdx = idx ^ 1;

        ChassisSnapshot cmd;
        bool estop;
        memcpy(&cmd, &cmdBuf[readIdx].snapshot, sizeof(cmd));
        estop = cmdBuf[readIdx].estopActive;

        if(estop || estopActive.load(std::memory_order_acquire)) {
            MotorControl::estop();
            motorPowerOn = false;
        } else {
            motorPowerOn = true;
            MotorControl::executeMotion(cmd);
        }
    }
}
