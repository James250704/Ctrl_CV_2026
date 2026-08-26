// ============================================================================
// Shoot — Slave 節點 (RP2350)
//
// 使用 SlaveComms 通用框架 + idleHook 機制：
//   Core 0 (loop)：   SlaveComms::update() — 收 Hub 控制幀
//   Core 1 (loop1)：  SlaveComms::runCore1() — 命令驅動 + 步進馬達增量移動
//
// 通訊：
//   Hub PIO SM1 (GP4 TX, GP5 RX) ↔ Shoot UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16
//
// 控制資料 (ShootPayload)：
//   upSpeed    (int8) — 上輪無刷馬達轉速，永久性速度設定 (-100~100)
//   downSpeed  (int8) — 下輪無刷馬達轉速，永久性速度設定 (-100~100)
//   pitch      (int8) — 俯仰步進馬達位置 (-100~100)
//   row        (int8) — 旋轉步進馬達位置 (-100~100)
//                        -100 = 逆時針 90°, 0 = 0°, 100 = 順時針 90°
//
// 硬體配置：
//   上輪無刷馬達: 硬體 PWM (50Hz servo-style PPM)
//   下輪無刷馬達: 硬體 PWM (50Hz servo-style PPM)
//   固定轉速無刷馬達: 硬體 PWM (50Hz, 固定脈寬)
//   俯仰步進馬達: STEP + DIR (A4988 / DRV8825 驅動器)
//   旋轉步進馬達: STEP + DIR (A4988 / DRV8825 驅動器)
//
// upSpeed/downSpeed 為永久性速度設定 — 一經設定即寫入 EEPROM 模擬區，
// 每次開機自動恢復上次設定的速度值，直到收到新的轉速指令為止。
// ============================================================================
#include "SlaveComms.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <cmath>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義 (依實際硬體接線修改)
// ============================================================================
namespace Pin {
// ——— 無刷馬達 (ESC PWM) ———
constexpr uint UP_BLDC_PWM = 2;
constexpr uint DOWN_BLDC_PWM = 3;
constexpr uint FIXED_BLDC_PWM = 4;

// ——— 步進馬達 (A4988/DRV8825) ———
constexpr uint PITCH_STEP = 5;
constexpr uint PITCH_DIR = 6;
constexpr uint ROW_STEP = 7;
constexpr uint ROW_DIR = 8;
constexpr uint STEPPER_ENABLE = 9; // 低電位致能
} // namespace Pin

// ============================================================================
// 步進馬達參數
// ============================================================================
namespace StepperConfig {
constexpr float STEP_PER_REV = 200.0f;                    // 1.8°/step (全步進)
constexpr float PULSE_PER_DEGREE = STEP_PER_REV / 360.0f; // ~0.556 step/deg
// ROW/PITCH 輸入映射：±100 → ±90°
constexpr float INPUT_TO_DEG = 90.0f / 100.0f; // 0.9 deg/input
} // namespace StepperConfig

// ============================================================================
// BLDC 速度持久化儲存（EEPROM 模擬）
//
// earlephilhower Arduino-Pico core 的 EEPROM 使用內部 Flash 模擬，
// begin() 分配 RAM 緩衝區，put() 寫入緩衝區，commit() 寫入 Flash。
// RP2350 支援 Flash 擦寫（wear leveling 由 core 內部處理）。
// ============================================================================
namespace SpeedStorage {
constexpr int EEPROM_SIZE = 16;    // 預留空間
constexpr int ADDR_UP_SPEED = 0;   // float, 4 bytes
constexpr int ADDR_DOWN_SPEED = 4; // float, 4 bytes

static float persistentUpSpeed = 0.0f;
static float persistentDownSpeed = 0.0f;

void init() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(ADDR_UP_SPEED, persistentUpSpeed);
    EEPROM.get(ADDR_DOWN_SPEED, persistentDownSpeed);
    // 若 flash 尚未初始化（全 0xFF），float 會是 NaN → 歸零
    if(std::isnan(persistentUpSpeed))
        persistentUpSpeed = 0.0f;
    if(std::isnan(persistentDownSpeed))
        persistentDownSpeed = 0.0f;
}

void saveUpSpeed(float speed) {
    persistentUpSpeed = speed;
    EEPROM.put(ADDR_UP_SPEED, speed);
    EEPROM.commit();
}

void saveDownSpeed(float speed) {
    persistentDownSpeed = speed;
    EEPROM.put(ADDR_DOWN_SPEED, speed);
    EEPROM.commit();
}

float getUpSpeed() { return persistentUpSpeed; }
float getDownSpeed() { return persistentDownSpeed; }
} // namespace SpeedStorage

// ============================================================================
// BLDC 馬達控制 (ESC)
//
// 硬體 PWM：clkdiv=125, wrap=19999 → 50Hz
// 脈衝寬度 1500μs = 中立點/停止, 2000μs = 全速
// ============================================================================
namespace BldcControl {
constexpr uint32_t PWM_CLKDIV = 125;
constexpr uint16_t PWM_WRAP = 19999;   // 20ms @ 1MHz
constexpr uint16_t PULSE_MID = 1500;   // 中立點/停止
constexpr uint16_t PULSE_MAX = 2000;   // 全速
constexpr uint16_t PULSE_FIXED = 1750; // 固定轉速無刷

static void setupPwm(uint gpio, uint16_t pulseUs) {
    uint slice = pwm_gpio_to_slice_num(gpio);
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    pwm_set_clkdiv(slice, PWM_CLKDIV);
    pwm_set_wrap(slice, PWM_WRAP);
    pwm_set_gpio_level(gpio, pulseUs);
    pwm_set_enabled(slice, true);
}

void init() {
    setupPwm(Pin::UP_BLDC_PWM, PULSE_MID);
    setupPwm(Pin::DOWN_BLDC_PWM, PULSE_MID);
    setupPwm(Pin::FIXED_BLDC_PWM, PULSE_FIXED);
}

static uint16_t speedToPulse(float speed) {
    if(speed == 0.0f)
        return PULSE_MID;
    float absSpd = fabsf(speed);
    if(absSpd > 100.0f)
        absSpd = 100.0f;
    uint16_t pulse = static_cast<uint16_t>(
        PULSE_MID + (absSpd / 100.0f) * (PULSE_MAX - PULSE_MID));
    return (pulse > PULSE_MAX) ? PULSE_MAX : pulse;
}

void setUpSpeed(float speed) {
    pwm_set_gpio_level(Pin::UP_BLDC_PWM, speedToPulse(speed));
}
void setDownSpeed(float speed) {
    pwm_set_gpio_level(Pin::DOWN_BLDC_PWM, speedToPulse(speed));
}
void estop() {
    pwm_set_gpio_level(Pin::UP_BLDC_PWM, PULSE_MID);
    pwm_set_gpio_level(Pin::DOWN_BLDC_PWM, PULSE_MID);
}
} // namespace BldcControl

// ============================================================================
// 步進馬達控制
//
// 設計：
//   - onCommand callback 設定目標位置 (setRowTarget / setPitchTarget)
//   - idleHook (每 spinloop 迭代呼叫) 執行 StepperControl::update()
//     逐步向目標移動，每次最多 MAX_STEPS_PER_CALL 步
//   - 已到達目標時 update() 零開銷 (diff==0 立即返回)
// ============================================================================
namespace StepperControl {
static int32_t pitchPosition = 0;
static int32_t rowPosition = 0;
static int32_t pitchTarget = 0;
static int32_t rowTarget = 0;

constexpr int MAX_STEPS_PER_CALL = 10;

void init() {
    gpio_init(Pin::PITCH_STEP);
    gpio_set_dir(Pin::PITCH_STEP, GPIO_OUT);
    gpio_init(Pin::PITCH_DIR);
    gpio_set_dir(Pin::PITCH_DIR, GPIO_OUT);
    gpio_init(Pin::ROW_STEP);
    gpio_set_dir(Pin::ROW_STEP, GPIO_OUT);
    gpio_init(Pin::ROW_DIR);
    gpio_set_dir(Pin::ROW_DIR, GPIO_OUT);

    gpio_init(Pin::STEPPER_ENABLE);
    gpio_set_dir(Pin::STEPPER_ENABLE, GPIO_OUT);
    gpio_put(Pin::STEPPER_ENABLE, 0); // 致能（低電位）
}

/// 輸入 -100~100 → 角度 → steps
static int32_t inputToSteps(float input) {
    if(input > 100.0f)
        input = 100.0f;
    if(input < -100.0f)
        input = -100.0f;
    float degrees = input * StepperConfig::INPUT_TO_DEG;
    return static_cast<int32_t>(
        roundf(degrees * StepperConfig::PULSE_PER_DEGREE));
}

void setRowTarget(float input) { rowTarget = inputToSteps(input); }
void setPitchTarget(float input) { pitchTarget = inputToSteps(input); }

/// 單軸增量移動，每次呼叫最多 maxSteps 步
static bool moveOneAxis(int32_t &pos, int32_t target, uint stepPin, uint dirPin,
                        int maxSteps) {
    int32_t diff = target - pos;
    if(diff == 0)
        return true;

    int32_t steps = static_cast<int32_t>(fabsf(static_cast<float>(diff)));
    if(steps > maxSteps)
        steps = maxSteps;

    gpio_put(dirPin, (diff > 0) ? 1 : 0);
    asm volatile("nop \n nop \n nop \n nop"); // DIR 建立時間

    for(int32_t i = 0; i < steps; i++) {
        gpio_put(stepPin, 1);
        asm volatile(
            "nop \n nop \n nop \n nop \n nop \n nop \n nop"); // >1μs @125MHz
        gpio_put(stepPin, 0);
        asm volatile("nop \n nop \n nop");
    }

    pos += (diff > 0) ? steps : -steps;
    return (pos == target);
}

/// 由 idleHook 每 spinloop 迭代呼叫
void update() {
    moveOneAxis(pitchPosition, pitchTarget, Pin::PITCH_STEP, Pin::PITCH_DIR,
                MAX_STEPS_PER_CALL);
    moveOneAxis(rowPosition, rowTarget, Pin::ROW_STEP, Pin::ROW_DIR,
                MAX_STEPS_PER_CALL);
}

void estop() {
    gpio_put(Pin::PITCH_STEP, 0);
    gpio_put(Pin::ROW_STEP, 0);
}
} // namespace StepperControl

// ============================================================================
// SlaveApp 回呼
// ============================================================================
static uint8_t tmFrameSeq = 0;       // 遙測幀序號
static MoveTelemetry pendingTm = {}; // Core 1 寫入 → Core 0 發送

static void onShootCommand(const ChassisSnapshot &cmd) {
    // 永久性速度：每次命令都儲存並套用（包括速度=0的清零指令）
    SpeedStorage::saveUpSpeed(cmd.shootUpSpeed);
    SpeedStorage::saveDownSpeed(cmd.shootDownSpeed);
    BldcControl::setUpSpeed(cmd.shootUpSpeed);
    BldcControl::setDownSpeed(cmd.shootDownSpeed);

    // 步進目標位置
    StepperControl::setPitchTarget(cmd.pitch);
    StepperControl::setRowTarget(cmd.row);

    // 構建遙測封包到暫存區，由 slave wrapper 在 Core 1 中交給 SlaveComms
    pendingTm = {};
    pendingTm.sync = 0xA5;
    pendingTm.msgType = MsgType::TELEMETRY;
    pendingTm.frameSeq = tmFrameSeq++;
    pendingTm.targetID = TargetID::HUB;
    // Shoot 遙測：復用 float 陣列
    // currentRps[0] = 上輪速度, currentRps[1] = 下輪速度
    pendingTm.currentRps[0] = cmd.shootUpSpeed;
    pendingTm.currentRps[1] = cmd.shootDownSpeed;
    pendingTm.targetRps[0] = cmd.shootUpSpeed;
    pendingTm.targetRps[1] = cmd.shootDownSpeed;
    pendingTm.mode = RobotMode::AUTO;
    pendingTm.estopFlag = 0;
    pendingTm.timestamp = time_us_64();
    pendingTm.crc16 = 0;
    pendingTm.crc16 =
        SignalUtils::calculateCrc(reinterpret_cast<const uint8_t *>(&pendingTm),
                                  sizeof(MoveTelemetry) - 2);
}

static void onShootEstop() {
    BldcControl::estop();
    StepperControl::estop();
}

// 前向宣告：shootCommandWrapper 定義在 slave 之後
static void shootCommandWrapper(const ChassisSnapshot &cmd);

static SlaveApp shootApp = {
    .targetID = TargetID::SHOOT,
    .onCommand = shootCommandWrapper, // wrapper: onShootCommand + 遙測發送
    .onEstop = onShootEstop,
};

// Wrapper：先執行 onShootCommand，再將遙測交給 SlaveComms
static SlaveComms slave(&shootApp);

static void shootCommandWrapper(const ChassisSnapshot &cmd) {
    onShootCommand(cmd);
    slave.prepareTelemetry(pendingTm);
}

// ============================================================================
// Core 1 idle hook — 每 spinloop 迭代空閒時驅動步進馬達
// ============================================================================
static void shootIdleHook() { StepperControl::update(); }

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    slave.begin();
    slave.setIdleHook(shootIdleHook);
}

void loop() { slave.update(); }

// ============================================================================
// Core 1
// ============================================================================
void setup1() {
    BldcControl::init();
    StepperControl::init();
    SpeedStorage::init();

    // 套用保留的速度值
    BldcControl::setUpSpeed(SpeedStorage::getUpSpeed());
    BldcControl::setDownSpeed(SpeedStorage::getDownSpeed());
    // 固定轉速無刷已在 BldcControl::init() 中設定
}

void loop1() { slave.runCore1(); }
