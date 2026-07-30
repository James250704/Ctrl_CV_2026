// ============================================================================
// Storage — Slave 節點 (RP2350)
//
// 使用 SlaveComms 通用框架 + idleHook 機制：
//   Core 0 (loop)：   SlaveComms::update() — 收 Hub 控制幀
//   Core 1 (loop1)：  SlaveComms::runCore1() — 命令驅動 + trigger 步進
//
// 通訊：
//   Hub PIO SM2 (GP10 TX, GP11 RX) ↔ Storage UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16
//
// 控制資料 (StoragePayload)：
//   motorChannel (uint8) — PCA9685 通道編號 0~15（選擇要控制的伺服通道）
//   sw           (uint8) — switch 0/1（控制所選通道伺服的位置）
//                           0 → 關閉位置 (500μs)
//                           1 → 開啟位置 (2500μs)
//   trigger      (uint8) — trigger 0/1（控制步進馬達旋轉）
//                           0 → 停止
//                           1 → 持續旋轉
//
// 硬體配置：
//   PCA9685: I2C1 (GP2=SDA, GP3=SCL), default address 0x40
//   PCA9685 通道 0~9 連接 10 顆伺服馬達
//   Trigger 步進馬達: STEP + DIR (GP4, GP5), A4988/DRV8825 驅動器
// ============================================================================
#include "SlaveComms.h"
#include <Arduino.h>
#include <hardware/pwm.h>
#include <hardware/i2c.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義 (依實際硬體接線修改)
// ============================================================================
namespace Pin {
// ——— PCA9685 (I2C1) ———
constexpr uint I2C_SDA = 2;
constexpr uint I2C_SCL = 3;

// ——— Trigger 步進馬達 ———
constexpr uint TRIGGER_STEP = 4;
constexpr uint TRIGGER_DIR  = 5;
constexpr uint STEPPER_ENABLE = 6;   // 低電位致能（與驅動器共用 Enable 腳位）
} // namespace Pin

// ============================================================================
// PCA9685 常數
// ============================================================================
namespace Pca9685 {
// I2C 位址
constexpr uint8_t ADDR         = 0x40;

// 暫存器
constexpr uint8_t MODE1        = 0x00;
constexpr uint8_t PRE_SCALE    = 0xFE;
constexpr uint8_t LED0_ON_L    = 0x06;   // 通道 0 ON 值低 byte
// 每通道佔 4 bytes: ON_L, ON_H, OFF_L, OFF_H
constexpr uint8_t CHAN_REG_STEP = 4;

// MODE1 位元
constexpr uint8_t MODE1_SLEEP  = 0x10;
constexpr uint8_t MODE1_AI     = 0x20;   // Auto-Increment
constexpr uint8_t MODE1_RESTART = 0x80;

// PWM 頻率 (50Hz for servo)
constexpr uint32_t PWM_FREQ    = 50;

// 計數器解析度 (12-bit: 0~4095)
constexpr uint16_t COUNTER_MAX = 4095;

// 伺服脈衝範圍 (μs)
constexpr uint16_t PULSE_MIN_US = 500;
constexpr uint16_t PULSE_MAX_US = 2500;

// 週期 (20ms = 20000μs)
constexpr uint32_t PERIOD_US   = 20000;

// 伺服通道數量 (使用通道 0~9)
constexpr uint8_t SERVO_CHANNEL_COUNT = 10;

// 伺服位置表：每個通道的關閉/開啟脈衝寬度 (μs)
// 索引 0~9 對應伺服通道 0~9
// 可依實際硬體調整每顆伺服的角度/行程
struct ServoConfig {
    uint16_t offPulseUs;   // sw=0 時的位置 (μs)
    uint16_t onPulseUs;    // sw=1 時的位置 (μs)
};

constexpr ServoConfig SERVO_PRESETS[SERVO_CHANNEL_COUNT] = {
    { 500,  2500 },  // CH0: 0° ~ 180°
    { 500,  2500 },  // CH1
    { 500,  2500 },  // CH2
    { 500,  2500 },  // CH3
    { 500,  2500 },  // CH4
    { 500,  2500 },  // CH5
    { 500,  2500 },  // CH6
    { 500,  2500 },  // CH7
    { 500,  2500 },  // CH8
    { 500,  2500 },  // CH9
};
} // namespace Pca9685

// ============================================================================
// PCA9685 驅動 (純 I2C, 無外部依賴)
// ============================================================================
namespace Pca9685Driver {
static i2c_inst_t* i2c = i2c1;

bool init() {
    // 初始化 I2C1
    i2c_init(i2c, 400000);   // 400kHz Fast Mode
    gpio_set_function(Pin::I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(Pin::I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(Pin::I2C_SDA);
    gpio_pull_up(Pin::I2C_SCL);

    // 檢查 PCA9685 是否存在
    uint8_t rx;
    int ret = i2c_read_blocking(i2c, Pca9685::ADDR, &rx, 1, false);
    if (ret < 0) {
        // PCA9685 無回應 — 仍繼續執行，不阻斷開機
        return false;
    }

    // 設定 MODE1: 離開 sleep、啟用 auto-increment
    uint8_t mode1 = 0x00;
    i2c_write_blocking(i2c, Pca9685::ADDR, &mode1, 1, false);
    // 使 auto-increment 生效
    uint8_t ai_buf[2] = { Pca9685::MODE1, Pca9685::MODE1_AI };
    i2c_write_blocking(i2c, Pca9685::ADDR, ai_buf + 1, 1, false);

    // 設定 PWM 頻率 (50Hz)
    // PRE_SCALE = round(osc_clock / (4096 × freq)) - 1
    // 內部震盪器 25MHz (typ)
    // PRE_SCALE = round(25e6 / (4096 × 50)) - 1 = round(122.07) - 1 = 121
    uint8_t preScale = 121;
    uint8_t preBuf[2] = { Pca9685::PRE_SCALE, preScale };
    i2c_write_blocking(i2c, Pca9685::ADDR, preBuf, 2, false);

    // 回到正常模式 (離開 sleep)
    mode1 = Pca9685::MODE1_AI;
    i2c_write_blocking(i2c, Pca9685::ADDR, &mode1, 1, false);
    busy_wait_us(500);  // 震盪器穩定時間

    return true;
}

/// 將脈衝寬度 (μs) 轉換為 PCA9685 的 OFF count (12-bit)
/// ON count 固定為 0 (從週期起點開始)
static uint16_t pulseUsToOffCount(uint16_t pulseUs) {
    if (pulseUs > Pca9685::PERIOD_US) pulseUs = Pca9685::PERIOD_US;
    // count = (pulseUs × 4096) / 20000
    uint32_t count = (static_cast<uint32_t>(pulseUs) * Pca9685::COUNTER_MAX) / Pca9685::PERIOD_US;
    // 取上限 (確保脈衝寬度 ≥ 請求值)
    if ((static_cast<uint32_t>(pulseUs) * Pca9685::COUNTER_MAX) % Pca9685::PERIOD_US != 0)
        count++;
    if (count > Pca9685::COUNTER_MAX) count = Pca9685::COUNTER_MAX;
    return static_cast<uint16_t>(count);
}

/// 設定單一通道的 PWM 脈衝寬度
/// @param channel  PCA9685 通道編號 (0~15)
/// @param pulseUs  脈衝寬度 (μs)，設為 0 則輸出恆低電位 (關閉)
void setChannelPulseUs(uint8_t channel, uint16_t pulseUs) {
    if (channel > 15) return;

    uint8_t reg = Pca9685::LED0_ON_L + channel * Pca9685::CHAN_REG_STEP;
    uint8_t buf[5];
    buf[0] = reg;       // 起始暫存器位址

    if (pulseUs == 0) {
        // 輸出恆低 (OFF count = 0, ON count = 4096 → 永遠 off)
        buf[1] = 0x00;  // LED0_ON_L
        buf[2] = 0x00;  // LED0_ON_H
        buf[3] = 0x00;  // LED0_OFF_L
        buf[4] = 0x10;  // LED0_OFF_H (bit 4 = OFF full = 1 → always off)
    } else {
        uint16_t offCount = pulseUsToOffCount(pulseUs);
        // ON = 0 (從週期起點開始)
        buf[1] = 0x00;          // LED0_ON_L
        buf[2] = 0x00;          // LED0_ON_H
        buf[3] = offCount & 0xFF;           // LED0_OFF_L
        buf[4] = (offCount >> 8) & 0x0F;    // LED0_OFF_H
    }

    i2c_write_blocking(i2c, Pca9685::ADDR, buf, 5, false);
}

/// 批次設定多個通道為相同脈衝寬度
void setAllChannelsPulseUs(uint16_t pulseUs) {
    uint16_t offCount = pulseUsToOffCount(pulseUs);
    for (uint8_t ch = 0; ch < 16; ch++) {
        uint8_t reg = Pca9685::LED0_ON_L + ch * Pca9685::CHAN_REG_STEP;
        uint8_t chBuf[5] = {
            reg,
            0x00, 0x00,
            static_cast<uint8_t>(offCount & 0xFF),
            static_cast<uint8_t>((offCount >> 8) & 0x0F)
        };
        i2c_write_blocking(i2c, Pca9685::ADDR, chBuf, 5, false);
    }
}
} // namespace Pca9685Driver

// ============================================================================
// 伺服狀態管理
//
// 每個通道記錄目前的開關狀態，PCA9685 的實際 PWM 輸出僅在
// 狀態變更時更新，避免不必要 I2C 寫入。
// ============================================================================
namespace ServoManager {
// 每通道當前 sw 狀態 (true=on, false=off)
static bool channelState[Pca9685::SERVO_CHANNEL_COUNT] = {false};

void init() {
    // 初始化所有伺服為關閉位置
    for (uint8_t ch = 0; ch < Pca9685::SERVO_CHANNEL_COUNT; ch++) {
        const auto& preset = Pca9685::SERVO_PRESETS[ch];
        Pca9685Driver::setChannelPulseUs(ch, preset.offPulseUs);
        channelState[ch] = false;
    }
}

/// 設定指定通道的 sw 狀態
/// @param channel  PCA9685 通道 (0~15)
/// @param sw       true=開啟位置, false=關閉位置
void setChannel(uint8_t channel, bool sw) {
    if (channel >= Pca9685::SERVO_CHANNEL_COUNT) return;

    // 狀態無變更則跳過 (去抖)
    if (channelState[channel] == sw) return;

    const auto& preset = Pca9685::SERVO_PRESETS[channel];
    uint16_t pulseUs = sw ? preset.onPulseUs : preset.offPulseUs;
    Pca9685Driver::setChannelPulseUs(channel, pulseUs);
    channelState[channel] = sw;
}

void estop() {
    // E-Stop: 所有伺服回到關閉位置
    for (uint8_t ch = 0; ch < Pca9685::SERVO_CHANNEL_COUNT; ch++) {
        if (channelState[ch]) {
            const auto& preset = Pca9685::SERVO_PRESETS[ch];
            Pca9685Driver::setChannelPulseUs(ch, preset.offPulseUs);
            channelState[ch] = false;
        }
    }
}
} // namespace ServoManager

// ============================================================================
// Trigger 步進馬達控制
//
// trigger=1 → 持續旋轉 (每次 idleHook 增量移動)
// trigger=0 → 停止
//
// 使用 idleHook 機制（與 Shoot 相同），在 Core 1 spinloop 空閒時
// 持續產生 STEP 脈衝。
// ============================================================================
namespace TriggerStepper {
// 步進馬達參數 (A4988/DRV8825, 全步進)
constexpr float STEP_PER_REV   = 200.0f;   // 1.8°/step

// trigger 狀態
static volatile bool triggerActive = false;

// 步進計數器 (僅用於追蹤位置)
static int32_t stepPosition = 0;

// 每次 idle 呼叫產生的步數
constexpr int STEPS_PER_IDLE = 5;

void init() {
    gpio_init(Pin::TRIGGER_STEP); gpio_set_dir(Pin::TRIGGER_STEP, GPIO_OUT);
    gpio_init(Pin::TRIGGER_DIR);  gpio_set_dir(Pin::TRIGGER_DIR,  GPIO_OUT);
    gpio_init(Pin::STEPPER_ENABLE);
    gpio_set_dir(Pin::STEPPER_ENABLE, GPIO_OUT);

    gpio_put(Pin::TRIGGER_DIR, 1);   // 預設正轉方向
    gpio_put(Pin::TRIGGER_STEP, 0);
    gpio_put(Pin::STEPPER_ENABLE, 0); // 致能（低電位）
}

/// 設定 trigger 狀態 (由 onCommand callback 呼叫)
void setTrigger(bool active) {
    triggerActive = active;
    if (!active) {
        // 停止：確保 STEP 為低電位，停止脈衝輸出
        gpio_put(Pin::TRIGGER_STEP, 0);
    }
}

/// idleHook 回呼 — 在 Core 1 spinloop 空閒時持續驅動
void update() {
    if (!triggerActive) return;

    for (int i = 0; i < STEPS_PER_IDLE; i++) {
        gpio_put(Pin::TRIGGER_STEP, 1);
        asm volatile("nop \n nop \n nop \n nop \n nop \n nop \n nop");
        gpio_put(Pin::TRIGGER_STEP, 0);
        asm volatile("nop \n nop \n nop \n nop \n nop \n nop");
        stepPosition++;
    }
}

void estop() {
    triggerActive = false;
    gpio_put(Pin::TRIGGER_STEP, 0);
}
} // namespace TriggerStepper

// ============================================================================
// 控制邏輯
// ============================================================================
namespace MotorControl {

static void init() {
    // 初始化 I2C PCA9685
    bool pcaOk = Pca9685Driver::init();
    // PCA9685 初始化失敗仍繼續執行 (可透過 trigger 步進馬達調試)

    // 初始化伺服狀態（設定初始脈衝）
    ServoManager::init();

    // 初始化 trigger 步進馬達
    TriggerStepper::init();

    (void)pcaOk;
}

static void executeMotion(const ChassisSnapshot& cmd) {
    // 1. motorChannel + sw: 控制 PCA9685 指定通道的伺服位置
    //    命令框架每次都會傳入完整的命令資料，
    //    motorChannel 為目標通道號 (0~15)
    //    sw=0 → 關閉位置, sw=1 → 開啟位置
    uint8_t channel = cmd.motorChannel;
    bool sw = (cmd.sw != 0);
    ServoManager::setChannel(channel, sw);

    // 2. trigger: 步進馬達旋轉控制
    //    trigger=1 → 持續旋轉, trigger=0 → 停止
    bool trigger = (cmd.trigger != 0);
    TriggerStepper::setTrigger(trigger);
}

static void estop() {
    ServoManager::estop();
    TriggerStepper::estop();
}
} // namespace MotorControl

// ============================================================================
// SlaveApp 回呼
// ============================================================================
static uint8_t tmFrameSeq = 0;  // 遙測幀序號
static MoveTelemetry pendingTm = {};  // Core 1 寫入 → Core 0 發送

static void onStorageEstop() {
    MotorControl::estop();
}

static void onStorageCommand(const ChassisSnapshot& cmd) {
    MotorControl::executeMotion(cmd);

    // 構建遙測封包到暫存區，由 storage wrapper 在 Core 1 中交給 SlaveComms
    pendingTm = {};
    pendingTm.sync       = 0xA5;
    pendingTm.msgType    = MsgType::TELEMETRY;
    pendingTm.frameSeq   = tmFrameSeq++;
    pendingTm.targetID   = TargetID::HUB;
    // Storage 遙測：復用 float 陣列
    // currentRps[0] = motorChannel, currentRps[1] = sw, currentRps[2] = trigger
    pendingTm.currentRps[0] = static_cast<float>(cmd.motorChannel);
    pendingTm.currentRps[1] = static_cast<float>(cmd.sw);
    pendingTm.currentRps[2] = static_cast<float>(cmd.trigger);
    pendingTm.mode     = RobotMode::AUTO;
    pendingTm.estopFlag = 0;
    pendingTm.timestamp = time_us_64();
    pendingTm.crc16 = 0;
    pendingTm.crc16 = SignalUtils::calculateCrc(
                           reinterpret_cast<const uint8_t*>(&pendingTm),
                           sizeof(MoveTelemetry) - 2);
}

// 前向宣告：storageCommandWrapper 定義在 slave 之後
static void storageCommandWrapper(const ChassisSnapshot& cmd);

static SlaveApp storageApp = {
    .targetID  = TargetID::STORAGE,
    .onCommand = storageCommandWrapper,  // wrapper: onStorageCommand + 遙測發送
    .onEstop   = onStorageEstop,
};

static SlaveComms slave(&storageApp);

// Wrapper：先執行 onStorageCommand，再將遙測交給 SlaveComms
static void storageCommandWrapper(const ChassisSnapshot& cmd) {
    onStorageCommand(cmd);
    slave.prepareTelemetry(pendingTm);
}

// ============================================================================
// Core 1 idle hook — 每 spinloop 迭代空閒時驅動 trigger 步進馬達
// ============================================================================
static void storageIdleHook() {
    TriggerStepper::update();
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    slave.begin();
    slave.setIdleHook(storageIdleHook);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    slave.update();
}

// ============================================================================
// Core 1
// ============================================================================
void setup1() {
    MotorControl::init();
}

void loop1() {
    slave.runCore1();
}
