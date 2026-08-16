// ============================================================================
// Move Slave — 四輪麥克納姆輪 開迴路 PWM 控制 (RP2040)
//
// 依 CLAUDE.md 雙核架構：Hub → COBS 控制幀 → Move UART0 收命令。
// 控制核心：純開迴路 PWM（無 PID、無編碼器回授）：
//   - Core 0 (loop):    SlaveComms::update() — 收 Hub 命令
//   - Core 1 (loop1):   SlaveComms::runCore1() — 執行開迴路 PWM 控制
//
// 通訊（CLAUDE.md §通訊協議）：
//   Hub PIO UART SM0 (GP2 TX, GP3 RX) ↔ Move UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16
//   接線：Hub SM0 TX(GP2) → Move UART0 RX(GP1)
//         Move UART0 TX(GP0) → Hub SM0 RX(GP3)
//
// 硬體腳位（新接線）：
//   馬達   PWM + DIR:   FL(10,11) FR(12,13) BL(8,9)  BR(6,7)
//   編碼器 PIO0 4 SM（僅讀取顯示，不參與控制）:
//                       FL(18,19) FR(20,21) BL(16,17) BR(14,15)
//
// 控制方式：
//   車身速度 [vx,vy,ω]（RPS，來自 Hub 的 ChassisSnapshot）
//      → 麥克納姆輪運動學 → 四輪目標線速度
//      → 線速度→duty 比例 (DUTY_PER_MPS) → 目標 duty%
//      → 線性斜率(ramp)平順爬升 → 硬體 PWM + DIR
//
// 實際控制測試：開機 500ms 無 Jetson 後，Hub 依遙控器 CH0/CH1/CH7
// 送 vx/vy/omega 至本 Slave，馬達即依此轉動。
// ============================================================================
#include "SlaveComms.h"
#include <Arduino.h>
#include <cmath>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義（新接線）
// ============================================================================
namespace Pin {
// 硬體 UART0 — 與 Hub 通訊（SlaveComms 內部處理 TX=GP0, RX=GP1）

// PWM + DIR for 4 wheels（新接線）
constexpr uint MOTOR_FL_PWM = 10;
constexpr uint MOTOR_FL_DIR = 11;
constexpr uint MOTOR_FR_PWM = 12;
constexpr uint MOTOR_FR_DIR = 13;
constexpr uint MOTOR_BL_PWM = 8;
constexpr uint MOTOR_BL_DIR = 9;
constexpr uint MOTOR_BR_PWM = 6;
constexpr uint MOTOR_BR_DIR = 7;

// Encoder pins（新接線，僅顯示）base pin，B = base+1
constexpr uint MOTOR_FL_ENC = 18;
constexpr uint MOTOR_FR_ENC = 20;
constexpr uint MOTOR_BL_ENC = 16;
constexpr uint MOTOR_BR_ENC = 14;
} // namespace Pin

// ============================================================================
// 參數
// ============================================================================
namespace MoveCfg {
// 硬體 PWM wrap（解析度）— 對齊 motor.h：wrap=1000, clkdiv=6.25 → ~20kHz
constexpr uint16_t PWM_WRAP = 1000;
// 開迴路 duty 斜率（每秒可變動 duty% 最大值）
constexpr float RAMP_RATE_PERCENT_PER_SEC = 100.0f;
// 線性斜率更新週期
constexpr uint32_t PWM_CTRL_MS = 5;
// 麥克納姆：車身半長/半寬/輪半徑 (m)
constexpr float HALF_LENGTH  = 0.25f;
constexpr float HALF_WIDTH   = 0.25f;
constexpr float WHEEL_RADIUS = 0.05f;
// 開迴路「車輪線速度(m/s) → duty%」比例（無回授近似，依實際調整）
constexpr float DUTY_PER_MPS = 60.0f;
} // namespace MoveCfg

// 每輪極性：1=依運動學符號，-1=馬達 forward 對映反轉（依實焊/線路校正）
// 指數 0=FL 1=FR 2=BL 3=BR
// 對齊你貼的參考主程式 g_pol 實作
static const int8_t kWheelPolarity[4] = {1, -1, 1, -1};

// ============================================================================
// 四輪狀態 — 開迴路
// ============================================================================
struct WheelState {
    uint pwmPin;
    uint dirPin;
    uint encPin;             // encoder base pin（僅顯示）
    float targetDuty = 0.0f; // 目標 duty% (-100~100)
    float outDuty    = 0.0f; // 實際輸出 duty%（斜坡逼近）
    int32_t encCount = 0;    // 編碼器計數（僅顯示）
};
static WheelState wheels[4];
static uint64_t lastTickUs = 0;   // Core 1 控制週期時間戳

// ============================================================================
// PIO 程式 — 四倍頻編碼器解碼（僅顯示用，不參與控制）
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
// 命令除錯快照 — Core 1 寫入，Core 0 印出（遵循專案規範）
// ============================================================================
static volatile float dbg_vx    = 0.0f;
static volatile float dbg_vy    = 0.0f;
static volatile float dbg_omega = 0.0f;
static volatile bool  dbg_estop = false;
static volatile uint32_t dbg_cmdCount = 0; // Core 1 實際執行 onCommand 的次數

// ============================================================================
// 硬體 PWM 輸出 + DIR（對齊 motor.h 行為）
//   - duty>=0 → DIR=LOW(正轉)；duty<0 → DIR=HIGH(反轉)，輸出 |duty|
//   - 輸入為 duty%（-100~100）；極性已在呼叫端乘以 kWheelPolarity[]
// ============================================================================
static void wheelSetPwm(uint pwmPin, uint dirPin, float dutyPercent) {
    bool fwd = (dutyPercent >= 0.0f);
    gpio_put(dirPin, fwd ? 0 : 1);   // LOW=正轉, HIGH=反轉
    float a = fabsf(dutyPercent);
    if (a > 100.0f) a = 100.0f;
    uint16_t level = static_cast<uint16_t>(a / 100.0f * MoveCfg::PWM_WRAP);
    pwm_set_gpio_level(pwmPin, level);
}

static void initWheelPwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    // 對齊 motor.h：clkdiv=6.25, wrap=1000 → ~20kHz，避免 50Hz 低頻震動
    pwm_config_set_clkdiv(&cfg, 6.25f);      // 125MHz / (1000+1) / 6.25 ≈ 20kHz
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
    pwm_config_set_wrap(&cfg, MoveCfg::PWM_WRAP);   // 1000
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(gpio, 0);
    pwm_set_enabled(slice, true);
}

// ============================================================================
// onMoveCommand — Core 1 收到命令時執行一次開迴路控制週期
// ============================================================================
static void onMoveCommand(const ChassisSnapshot& cmd) {
    // 記錄收到的命令供 Core 0 印出（實際控制測試用）
    dbg_cmdCount++;
    dbg_vx    = cmd.vx;
    dbg_vy    = cmd.vy;
    dbg_omega = cmd.omega;
    dbg_estop = false;

    // 車身速度 (RPS) → 各輪目標線速度 (m/s)
    //   輪線速度 = 輪轉速(rad/s) × 輪半徑
    //   wx 帶負號：校正前後(vx)方向（只影響前後，不影響左右/旋轉）
    float wx = -cmd.vx * TWO_PI;   // rad/s (vx 方向校正)
    float wy =  cmd.vy * TWO_PI;   // rad/s
    float K  = MoveCfg::HALF_LENGTH + MoveCfg::HALF_WIDTH;

    // 麥克納姆「標準正向」公式（輪線速度，m/s）— 對齊你貼的 kinToWheel
    //   座標系：x=車頭+，y=車右+，ω 逆時針=+
    //   FL = vx - vy - K·ω ; FR = vx + vy + K·ω
    //   BL = vx + vy - K·ω ; BR = vx - vy + K·ω
    //   方向差異（實際接線）一律用 kWheelPolarity[] 校正，不改運動學公式
    float lin[4];
    lin[0] = (wx - wy - K * cmd.omega) * MoveCfg::WHEEL_RADIUS; // FL
    lin[1] = (wx + wy + K * cmd.omega) * MoveCfg::WHEEL_RADIUS; // FR
    lin[2] = (wx + wy - K * cmd.omega) * MoveCfg::WHEEL_RADIUS; // BL
    lin[3] = (wx - wy + K * cmd.omega) * MoveCfg::WHEEL_RADIUS; // BR

    // 線速度 → 目標 duty%
    for (int i = 0; i < 4; i++) {
        float duty = lin[i] * MoveCfg::DUTY_PER_MPS;
        if (duty > 100.0f) duty = 100.0f;
        if (duty < -100.0f) duty = -100.0f;
        wheels[i].targetDuty = duty;
    }

    // 開迴路 slope 逼近 + 寫入 PWM（每次控制幀執行）
    uint64_t nowUs = time_us_64();
    float dt = static_cast<float>(nowUs - lastTickUs) * 1e-6f;
    if (dt < 1e-4f) dt = 1e-4f;
    if (dt > 0.05f) dt = 0.05f;
    lastTickUs = nowUs;
    float maxStep = MoveCfg::RAMP_RATE_PERCENT_PER_SEC * dt;
    for (int i = 0; i < 4; i++) {
        float d = wheels[i].targetDuty - wheels[i].outDuty;
        if (d > maxStep) d = maxStep;
        if (d < -maxStep) d = -maxStep;
        wheels[i].outDuty += d;
        wheelSetPwm(wheels[i].pwmPin, wheels[i].dirPin,
                    wheels[i].outDuty * kWheelPolarity[i]);
    }
}

// ============================================================================
// onMoveEstop — 立即歸零所有 PWM
// ============================================================================
static void onMoveEstop() {
    for (int i = 0; i < 4; i++) {
        wheels[i].targetDuty = 0.0f;
        wheels[i].outDuty    = 0.0f;
        wheelSetPwm(wheels[i].pwmPin, wheels[i].dirPin, 0.0f);
    }
    dbg_estop = true;
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    // ---- 本機除錯輸出 (USB CDC) ----
    Serial.begin(115200);
    delay(200);
    Serial.println("[MOVE] 開迴路 PWM 啟動（等待 Hub 命令）");

    // ---- 通訊（SlaveComms 內部處理 Serial1 + ChassisComms）----
    slave.begin();

    // ---- 馬達 PWM + DIR 初始化 ----
    constexpr uint pwmPins[4] = { Pin::MOTOR_FL_PWM, Pin::MOTOR_FR_PWM,
                                  Pin::MOTOR_BL_PWM, Pin::MOTOR_BR_PWM };
    constexpr uint dirPins[4] = { Pin::MOTOR_FL_DIR, Pin::MOTOR_FR_DIR,
                                  Pin::MOTOR_BL_DIR, Pin::MOTOR_BR_DIR };
    constexpr uint encPins[4] = { Pin::MOTOR_FL_ENC, Pin::MOTOR_FR_ENC,
                                  Pin::MOTOR_BL_ENC, Pin::MOTOR_BR_ENC };

    for (int i = 0; i < 4; i++) {
        initWheelPwm(pwmPins[i]);
        wheels[i].pwmPin = pwmPins[i];
        wheels[i].dirPin = dirPins[i];
        wheels[i].encPin = encPins[i];
        wheels[i].targetDuty = 0.0f;
        wheels[i].outDuty    = 0.0f;

        // DIR pin (輸出，預設正向)
        gpio_init(dirPins[i]);
        gpio_set_dir(dirPins[i], GPIO_OUT);
        gpio_put(dirPins[i], 1);
    }

    // ---- PIO 編碼器初始化（僅顯示）----
    PIO encPio = pio0;
    uint encOffset = pio_add_program(encPio, &pio_encoder_program);
    for (int i = 0; i < 4; i++) {
        uint pinA = encPins[i];
        uint pinB = encPins[i] + 1;
        pio_gpio_init(encPio, pinA); pio_gpio_init(encPio, pinB);
        gpio_set_function(pinA, GPIO_FUNC_PIO0);
        gpio_set_function(pinB, GPIO_FUNC_PIO0);

        pio_sm_config c = pio_get_default_sm_config();
        sm_config_set_wrap(&c, encOffset, encOffset + pio_encoder_program.length - 1);
        sm_config_set_in_pins(&c, pinA);
        sm_config_set_jmp_pin(&c, pinA);
        sm_config_set_in_pin_count(&c, 2);
        sm_config_set_clkdiv(&c, 1.0f);
        pio_sm_init(encPio, i, encOffset, &c);
        pio_sm_set_enabled(encPio, i, true);
    }
}

void loop() {
    slave.update();

    // 每 100ms 印一次收到的命令與實際輸出（Core 0，safe 用 Serial）
    static uint32_t lastPrint = 0;
    uint32_t nowCfg = millis();
    if (nowCfg - lastPrint >= 100) {
        lastPrint = nowCfg;
        // 診斷資訊：距上次有效命令 ms、有效命令總數、Core1 onCmd 次數
        uint32_t since = slave.msSinceLastValidCmd();
        uint32_t cnt   = slave.validCmdCount();
        Serial.printf("[MOVE] sinceCmd=%ums cnt=%u onCmd=%u\n",
                      since, cnt, dbg_cmdCount);
        if (dbg_estop) {
            Serial.println("[MOVE] ** E-STOP active **");
        } else {
            Serial.printf("[MOVE] vx=%+.2f vy=%+.2f omega=%+.2f | "
                          "duty=%+.0f,%+.0f,%+.0f,%+.0f%%\n",
                          dbg_vx, dbg_vy, dbg_omega,
                          wheels[0].outDuty, wheels[1].outDuty,
                          wheels[2].outDuty, wheels[3].outDuty);
        }
    }
}

// ============================================================================
// Core 1 — 自旋等待 SlaveComms 派發命令，執行開迴路 PWM 控制
// ============================================================================
void setup1() {
    // 無需額外初始化
}

void loop1() {
    slave.runCore1();
}
