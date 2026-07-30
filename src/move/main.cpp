// ============================================================================
// Move Slave — 四輪麥克納姆輪控制韌體 (RP2350)
//
// 根據 CLAUDE.md 雙核架構規範：
//   Core 0 (loop):    SlaveComms::update() — 收 Hub 控制幀 → 解命令
//   Core 1 (loop1):   SlaveComms::runCore1() — 自旋等待命令，執行
//                      Mecanum 運動學解算 + 四輪獨立 PID 控制
//
// 通訊（CLAUDE.md §通訊協議）：
//   Hub PIO UART SM0 (GP6 TX, GP7 RX) ↔ Move UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16
//   接線：Hub SM0 TX(GP6) → Move RX(GP1)
//         Move TX(GP0) → Hub SM0 RX(GP7)
//
// 硬體配置（每輪）：
//   - 硬體 PWM + DIR pin（取代 PIO PWM，CLAUDE.md 腳位規格）
//   - 一組 PIO Quadrature Encoder (1 個 SM)
//   - 四輪總計：4×Encoder = 4 SM (PIO0)
//   - BMI270 IMU on I2C0 (GP4=SDA, GP5=SCL)
// ============================================================================
#include "SlaveComms.h"
#include <Arduino.h>
#include <cmath>
#include <hardware/clocks.h>
#include <hardware/i2c.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <hardware/timer.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義 (依 CLAUDE.md 硬體規格)
// ============================================================================
namespace Pin {
// 硬體 UART0 — 與 Hub 通訊
// 預設腳位 TX=GP0, RX=GP1（SlaveComms 內部已處理）
// 接線：Hub PIO SM0 TX(GP2) → Move RX(GP1)
//       Move TX(GP0) → Hub PIO SM0 RX(GP3)

// PWM+DIR for 4 wheels
// FL — Front Left
constexpr uint MOTOR_FL_PWM = 6;
constexpr uint MOTOR_FL_DIR = 7;
// FR — Front Right
constexpr uint MOTOR_FR_PWM = 8;
constexpr uint MOTOR_FR_DIR = 9;
// BL — Back Left
constexpr uint MOTOR_BL_PWM = 10;
constexpr uint MOTOR_BL_DIR = 11;
// BR — Back Right
constexpr uint MOTOR_BR_PWM = 12;
constexpr uint MOTOR_BR_DIR = 13;

// Encoder pins (CLAUDE.md spec)
// FL: A=GP14, B=GP15
// FR: A=GP16, B=GP17
// BL: A=GP18, B=GP19
// BR: A=GP20, B=GP21
constexpr uint MOTOR_FL_ENC_A = 14;
constexpr uint MOTOR_FL_ENC_B = 15;
constexpr uint MOTOR_FR_ENC_A = 16;
constexpr uint MOTOR_FR_ENC_B = 17;
constexpr uint MOTOR_BL_ENC_A = 18;
constexpr uint MOTOR_BL_ENC_B = 19;
constexpr uint MOTOR_BR_ENC_A = 20;
constexpr uint MOTOR_BR_ENC_B = 21;

// BMI270 IMU I2C
constexpr uint IMU_SDA = 4;
constexpr uint IMU_SCL = 5;
} // namespace Pin

// ============================================================================
// 運動學參數
// ============================================================================
namespace Kinematics {
// 車身半長 (m) — 依實際車體修改
constexpr float HALF_LENGTH   = 0.25f;
// 車身半寬 (m)
constexpr float HALF_WIDTH    = 0.25f;
// 輪半徑 (m)
constexpr float WHEEL_RADIUS  = 0.05f;
// 從 (vx, vy, omega) 算到各輪期望轉速 (rad/s)
constexpr float L_PLUS_W      = HALF_LENGTH + HALF_WIDTH;
constexpr float INV_R         = 1.0f / WHEEL_RADIUS;
// Feedforward 增益：rps → PWM duty 的開迴路增益
constexpr float kFF           = 7.0f;
// 車身加速度 ramping 限制 (RPS/s²)
constexpr float ACCEL_LIMIT   = 20.0f;

inline void computeWheelTargets(float vx, float vy, float omega,
                                float& fl, float& fr, float& rl, float& rr) {
    float lwOmega = L_PLUS_W * omega;
    fl = (vx - vy - lwOmega) * INV_R;
    fr = (vx + vy + lwOmega) * INV_R;
    rl = (vx + vy - lwOmega) * INV_R;
    rr = (vx - vy + lwOmega) * INV_R;
}

inline float feedforward(float targetRps) { return targetRps / kFF; }
} // namespace Kinematics

// ============================================================================
// PID + Feedforward 控制器
// ============================================================================
class PidController {
public:
    float kp = 1.5f, ki = 0.0f, kd = 0.0f;
    float outLimit = RobotConfig::PID_OUT_LIMIT;
    float integralLimit = 5.0f;

    void reset() {
        integral_ = 0.0f;
        prevError_ = 0.0f;
        firstRun_ = true;
    }

    float update(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        if (firstRun_) { prevError_ = error; firstRun_ = false; }

        float pTerm = kp * error;
        integral_ += ki * error * dt;
        if (integral_ > integralLimit) integral_ = integralLimit;
        if (integral_ < -integralLimit) integral_ = -integralLimit;
        float dTerm = kd * (error - prevError_) / dt;
        prevError_ = error;

        float pidOut = pTerm + integral_ + dTerm;
        float ffOut = Kinematics::feedforward(setpoint);
        float output = pidOut + ffOut;

        if (output > outLimit) output = outLimit;
        if (output < -outLimit) output = -outLimit;
        return output;
    }

private:
    float integral_ = 0.0f;
    float prevError_ = 0.0f;
    bool firstRun_ = true;
};

// ============================================================================
// RampedTarget — 車身速度層加速度漸變
// ============================================================================
class RampedTarget {
public:
    explicit RampedTarget(float maxDeltaPerSec) : maxDelta_(maxDeltaPerSec) {}
    void reset(float value = 0.0f) { previous_ = value; }
    float update(float target, float dt) {
        float maxStep = maxDelta_ * dt;
        float diff = target - previous_;
        if (diff > maxStep) diff = maxStep;
        if (diff < -maxStep) diff = -maxStep;
        previous_ += diff;
        return previous_;
    }
private:
    float maxDelta_;
    float previous_ = 0.0f;
};

// ============================================================================
// 四輪狀態
// ============================================================================
struct WheelState {
    uint pwmPin;         // 硬體 PWM GPIO
    uint dirPin;         // DIR GPIO
    uint encSm;
    PIO pioBlock;
    PidController pid;
    volatile int32_t encoderCount;
    float currentRps = 0.0f;
    float targetRps = 0.0f;
    float pwmOutput = 0.0f;
};
static WheelState wheels[4];

// ============================================================================
// PIO 程式 — 四倍頻編碼器解碼
// ============================================================================
static const uint16_t pio_encoder_instructions[] = {
    0x2020, // 0: wait 0 pin, 0
    0x20a0, // 1: wait 1 pin, 0
    0x2021, // 2: wait 0 pin, 1
    0x20a1, // 3: wait 1 pin, 1
    0x0040, // 4: jmp x--, 0
};
static const pio_program_t pio_encoder_program = {
    .instructions = pio_encoder_instructions,
    .length = 5, .origin = -1,
};

// ============================================================================
// 前向宣告
// ============================================================================
static void onMoveCommand(const ChassisSnapshot& cmd);
static void onMoveEstop();
static void readEncoders(float dt);
static void wheelSetPwm(uint pwmPin, uint dirPin, float value);

// ============================================================================
// SlaveApp 回呼實作 — 由 SlaveComms 在 Core 1 中呼叫
// ============================================================================
static SlaveApp moveApp = {
    .targetID  = TargetID::MOVE,
    .onCommand = onMoveCommand,
    .onEstop   = onMoveEstop,
};

static SlaveComms slave(&moveApp);

// ============================================================================
// 運動控制狀態（Core 1 專用）
// ============================================================================
static RampedTarget rampVx(Kinematics::ACCEL_LIMIT);
static RampedTarget rampVy(Kinematics::ACCEL_LIMIT);
static RampedTarget rampOmega(Kinematics::ACCEL_LIMIT);
static bool         rampInitialized = false;
static bool         prevEstop       = false;
static uint64_t     lastTickUs      = 0;
static uint8_t      tmFrameSeq      = 0;  // 遙測幀序號

// ============================================================================
// wheelSetPwm — 設定一輪的 PWM duty + DIR 方向
//
// value > 0 → forward (DIR=1), value < 0 → reverse (DIR=0)
// PWM duty 根據 PID_OUT_LIMIT 按比例映射至 MOTOR_MAX_DUTY
// ============================================================================
static void wheelSetPwm(uint pwmPin, uint dirPin, float value) {
    gpio_put(dirPin, (value >= 0.0f) ? 1 : 0);
    float absVal = fabsf(value);
    uint16_t duty = (absVal > RobotConfig::PID_OUT_LIMIT) ? RobotConfig::MOTOR_MAX_DUTY
        : static_cast<uint16_t>(absVal / RobotConfig::PID_OUT_LIMIT * RobotConfig::MOTOR_MAX_DUTY);
    pwm_set_gpio_level(pwmPin, duty);
}

// ============================================================================
// onMoveCommand — Core 1 收到命令時執行一次控制週期
// ============================================================================
static void onMoveCommand(const ChassisSnapshot& cmd) {
    if (!rampInitialized) {
        rampVx.reset();
        rampVy.reset();
        rampOmega.reset();
        lastTickUs = time_us_64();
        rampInitialized = true;
    }

    // E-Stop falling edge：復原瞬間重置 ramper
    if (prevEstop) {
        prevEstop = false;
        rampVx.reset();
        rampVy.reset();
        rampOmega.reset();
        lastTickUs = time_us_64();
    }

    // 計算 dt
    uint64_t now = time_us_64();
    float dt = static_cast<float>(now - lastTickUs) * 1e-6f;
    if (dt < 1e-6f) dt = 1e-6f;
    if (dt > 5e-3f) dt = 5e-3f;
    lastTickUs = now;

    // 車身速度 ramping
    float rampedVx   = rampVx.update(cmd.vx, dt);
    float rampedVy   = rampVy.update(cmd.vy, dt);
    float rampedOmega = rampOmega.update(cmd.omega, dt);

    // Mecanum 運動學解算
    float flTarget, frTarget, rlTarget, rrTarget;
    Kinematics::computeWheelTargets(rampedVx, rampedVy, rampedOmega,
                                    flTarget, frTarget, rlTarget, rrTarget);

    // 讀取編碼器
    readEncoders(dt);

    // PID + Feedforward
    float pwmFL = wheels[0].pid.update(flTarget, wheels[0].currentRps, dt);
    float pwmFR = wheels[1].pid.update(frTarget, wheels[1].currentRps, dt);
    float pwmRL = wheels[2].pid.update(rlTarget, wheels[2].currentRps, dt);
    float pwmRR = wheels[3].pid.update(rrTarget, wheels[3].currentRps, dt);

    // NaN 安全檢查
    if (std::isnan(pwmFL) || std::isinf(pwmFL) ||
        std::isnan(pwmFR) || std::isinf(pwmFR) ||
        std::isnan(pwmRL) || std::isinf(pwmRL) ||
        std::isnan(pwmRR) || std::isinf(pwmRR)) {
        onMoveEstop();
        return;
    }

    // 寫入硬體 PWM+DIR
    wheelSetPwm(wheels[0].pwmPin, wheels[0].dirPin, pwmFL);
    wheelSetPwm(wheels[1].pwmPin, wheels[1].dirPin, pwmFR);
    wheelSetPwm(wheels[2].pwmPin, wheels[2].dirPin, pwmRL);
    wheelSetPwm(wheels[3].pwmPin, wheels[3].dirPin, pwmRR);

    // 更新狀態
    wheels[0].targetRps = flTarget; wheels[0].pwmOutput = pwmFL;
    wheels[1].targetRps = frTarget; wheels[1].pwmOutput = pwmFR;
    wheels[2].targetRps = rlTarget; wheels[2].pwmOutput = pwmRL;
    wheels[3].targetRps = rrTarget; wheels[3].pwmOutput = pwmRR;

    // 構建遙測封包，交由 Core 0 定時發送
    MoveTelemetry tm = {};
    tm.sync       = 0xA5;
    tm.msgType    = MsgType::TELEMETRY;
    tm.frameSeq   = tmFrameSeq++;
    tm.targetID   = TargetID::HUB;
    for (int i = 0; i < 4; i++) {
        tm.currentRps[i] = wheels[i].currentRps;
        tm.targetRps[i]  = wheels[i].targetRps;
        tm.pwmOutput[i]  = wheels[i].pwmOutput;
    }
    tm.cmdVx    = cmd.vx;
    tm.cmdVy    = cmd.vy;
    tm.cmdOmega = cmd.omega;
    tm.mode     = RobotMode::AUTO;
    tm.estopFlag = 0;
    tm.timestamp = time_us_64();
    tm.crc16 = 0;
    tm.crc16 = SignalUtils::calculateCrc(
                   reinterpret_cast<const uint8_t*>(&tm),
                   sizeof(MoveTelemetry) - 2);
    slave.prepareTelemetry(tm);
}

// ============================================================================
// onMoveEstop — 立即歸零所有 PWM
// ============================================================================
static void onMoveEstop() {
    for (int i = 0; i < 4; i++) {
        pwm_set_gpio_level(wheels[i].pwmPin, 0);
        wheels[i].pid.reset();
        wheels[i].currentRps = 0.0f;
        wheels[i].targetRps = 0.0f;
        wheels[i].pwmOutput = 0.0f;
    }
    rampVx.reset();
    rampVy.reset();
    rampOmega.reset();
    prevEstop = true;
}

// ============================================================================
// 讀取編碼器並計算各輪當前轉速
// ============================================================================
static void readEncoders(float dt) {
    constexpr float ENC_COUNTS_PER_REV = 4.0f * 1200.0f;
    constexpr float INV_ENC = 1.0f / ENC_COUNTS_PER_REV;
    static int32_t prevCount[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++) {
        int32_t raw = wheels[i].encoderCount;
        int32_t delta = raw - prevCount[i];
        prevCount[i] = raw;
        wheels[i].currentRps = static_cast<float>(delta) * INV_ENC / dt;
    }
}

// ============================================================================
// BMI270 IMU 驅動
// ============================================================================
namespace Bmi270 {
    static i2c_inst_t* i2c = i2c0;

    constexpr uint8_t ADDR = 0x68;         // BMI270 I2C address (ADDR=0)
    constexpr uint8_t REG_CHIP_ID  = 0x00;
    constexpr uint8_t REG_PWR_CONF = 0x7C;
    constexpr uint8_t REG_INIT_CTRL = 0x59;
    constexpr uint8_t REG_ACC_CONF = 0x40;
    constexpr uint8_t REG_GYR_CONF = 0x42;
    constexpr uint8_t REG_ACC_DATA = 0x12;  // 6 bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    constexpr uint8_t REG_GYR_DATA = 0x18;  // 6 bytes

    struct ImuData {
        int16_t accX, accY, accZ;
        int16_t gyrX, gyrY, gyrZ;
        bool valid;
    };

    static bool initialized = false;

    static uint8_t readReg(uint8_t reg) {
        uint8_t val;
        i2c_write_blocking(i2c, ADDR, &reg, 1, true);
        i2c_read_blocking(i2c, ADDR, &val, 1, false);
        return val;
    }

    static void writeReg(uint8_t reg, uint8_t val) {
        uint8_t buf[2] = {reg, val};
        i2c_write_blocking(i2c, ADDR, buf, 2, false);
    }

    bool init() {
        i2c_init(i2c, 400000);
        gpio_set_function(Pin::IMU_SDA, GPIO_FUNC_I2C);
        gpio_set_function(Pin::IMU_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(Pin::IMU_SDA);
        gpio_pull_up(Pin::IMU_SCL);

        uint8_t chipId = readReg(REG_CHIP_ID);
        if (chipId != 0x24) {  // BMI270 chip ID
            initialized = false;
            return false;
        }

        // Power on
        writeReg(REG_PWR_CONF, 0x00);
        busy_wait_us(1000);

        // Accel: ±2g, normal mode, 100Hz ODR
        writeReg(REG_ACC_CONF, 0x0A);

        // Gyro: ±2000dps, normal mode, 100Hz ODR
        writeReg(REG_GYR_CONF, 0x0A);

        initialized = true;
        return true;
    }

    ImuData read() {
        ImuData data = {0, 0, 0, 0, 0, 0, false};
        if (!initialized) return data;

        uint8_t reg = REG_ACC_DATA;
        uint8_t buf[12];
        i2c_write_blocking(i2c, ADDR, &reg, 1, true);
        i2c_read_blocking(i2c, ADDR, buf, 12, false);

        data.accX = static_cast<int16_t>(buf[0] | (buf[1] << 8));
        data.accY = static_cast<int16_t>(buf[2] | (buf[3] << 8));
        data.accZ = static_cast<int16_t>(buf[4] | (buf[5] << 8));
        data.gyrX = static_cast<int16_t>(buf[6] | (buf[7] << 8));
        data.gyrY = static_cast<int16_t>(buf[8] | (buf[9] << 8));
        data.gyrZ = static_cast<int16_t>(buf[10] | (buf[11] << 8));
        data.valid = true;
        return data;
    }
} // namespace Bmi270

// ============================================================================
// 硬體 PWM 初始化輔助 — 為特定 GPIO 設定 PWM 參數
// ============================================================================
static void initHardwarePwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);

    pwm_config cfg = pwm_get_default_config();
    // 設定 clkdiv 與 wrap 以獲得約 25kHz PWM
    // System clock 150MHz, wrap=40000, div=150
    // PWM freq = 150e6 / (150 * 40000) ≈ 25kHz
    pwm_config_set_clkdiv(&cfg, 150.0f);
    pwm_config_set_wrap(&cfg, RobotConfig::MOTOR_MAX_DUTY);
    pwm_init(slice, &cfg, false);
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    // ---- 通訊（SlaveComms 內部處理 Serial1 + ChassisComms）----
    slave.begin();

    // ---- 硬體 PWM 初始化 ----'
    // PWM pins: FL=6, FR=8, BL=10, BR=12
    constexpr uint pwmPins[4] = {
        Pin::MOTOR_FL_PWM, Pin::MOTOR_FR_PWM,
        Pin::MOTOR_BL_PWM, Pin::MOTOR_BR_PWM
    };
    // DIR pins: FL=7, FR=9, BL=11, BR=13
    constexpr uint dirPins[4] = {
        Pin::MOTOR_FL_DIR, Pin::MOTOR_FR_DIR,
        Pin::MOTOR_BL_DIR, Pin::MOTOR_BR_DIR
    };

    for (int i = 0; i < 4; i++) {
        // 初始化硬體 PWM
        initHardwarePwm(pwmPins[i]);
        pwm_set_gpio_level(pwmPins[i], 0);

        // 初始化 DIR pin（輸出，預設正向）
        gpio_init(dirPins[i]);
        gpio_set_dir(dirPins[i], GPIO_OUT);
        gpio_put(dirPins[i], 1);

        // 填入 WheelState
        wheels[i].pwmPin = pwmPins[i];
        wheels[i].dirPin = dirPins[i];
        wheels[i].pid.reset();

        // 啟動 PWM（使能後才開始輸出）
        pwm_set_enabled(pwm_gpio_to_slice_num(pwmPins[i]), true);
    }

    // ---- PIO Encoder 初始化 (block0) ----
    PIO encPio = pio0;
    uint encOffset = pio_add_program(encPio, &pio_encoder_program);
    for (int i = 0; i < 4; i++) {
        wheels[i].encSm = static_cast<uint>(i);

        uint pinA, pinB;
        switch (i) {
            case 0: pinA = Pin::MOTOR_FL_ENC_A; pinB = Pin::MOTOR_FL_ENC_B; break;
            case 1: pinA = Pin::MOTOR_FR_ENC_A; pinB = Pin::MOTOR_FR_ENC_B; break;
            case 2: pinA = Pin::MOTOR_BL_ENC_A; pinB = Pin::MOTOR_BL_ENC_B; break;
            case 3: pinA = Pin::MOTOR_BR_ENC_A; pinB = Pin::MOTOR_BR_ENC_B; break;
            default: pinA = 0; pinB = 0; break;
        }

        pio_gpio_init(encPio, pinA); pio_gpio_init(encPio, pinB);
        gpio_set_function(pinA, GPIO_FUNC_PIO0);
        gpio_set_function(pinB, GPIO_FUNC_PIO0);

        pio_sm_config encCfg = pio_get_default_sm_config();
        sm_config_set_wrap(&encCfg, encOffset, encOffset + pio_encoder_program.length - 1);
        sm_config_set_in_pins(&encCfg, pinA);
        sm_config_set_jmp_pin(&encCfg, pinA);
        sm_config_set_in_pin_count(&encCfg, 2);
        sm_config_set_clkdiv(&encCfg, 1.0f);

        pio_sm_init(encPio, i, encOffset, &encCfg);
        pio_sm_set_enabled(encPio, i, true);
    }

    // ---- BMI270 IMU 初始化 ----
    if (Bmi270::init()) {
        // IMU 初始化成功（可透過 SlaveComms 記錄或忽略）
    }
}

void loop() {
    slave.update();
}

// ============================================================================
// Core 1 — 自旋等待 SlaveComms 派發命令，執行運動控制
// ============================================================================
void setup1() {
    // 無需額外初始化
}

void loop1() {
    slave.runCore1();
}
