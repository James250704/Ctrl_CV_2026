// ============================================================================
// Move Slave — 四輪麥克納姆輪，四輪速度閉迴路 PID (RP2350)
//
// 依 CLAUDE.md 雙核架構：Hub → COBS 控制幀 → Move UART0 收命令。
//
// ### 控制核心（Core 1, onMoveCommand）###
//   車身速度 [vx,vy,ω] (RPS) → 麥克納姆運動學 → 四輪目標線速度(m/s)
//     → 目標輪轉速 targetRps (RPS = rev/s)
//   每輪: 速度閉迴路 PID（編碼器 RPS 回授 → 前饋 ff + PI + 抗飽和 → duty%）
//   → 硬體 PWM + DIR
//   四輪 PID 由實測（綁 PIO 架構）依序調出，見 kPidCfg[4]。
//
// ### 通訊 ###
//   Hub PIO UART SM0 (GP2 TX, GP3 RX) ↔ Move UART0 (TX=GP0, RX=GP1)
//   921600 baud, COBS 封幀 + CRC16, SlaveComms 框架
//
// ### 編碼器 ###
//   PIO0 4 個 SM，每 SM 一顆輪，硬體採樣相位、push 到 RX FIFO（無 GPIO 中斷）。
//   主迴圈( Core 1 control 週期) drain FIFO + Gray 表累計，並套
//   kEncReadPolarity 統一方向。PIO 取樣沿用實測驗證成功之設定：
//     pio_gpio_init + `in pins,2` left-shift + clkdiv=5000 濾回彈。
// ============================================================================
#include "SlaveComms.h"
#include "quad_phase.pio.h"
#include <Arduino.h>
#include <cmath>
#include <hardware/gpio.h>
#include <hardware/pio.h>
#include <hardware/pwm.h>
#include <pico/stdlib.h>

// ============================================================================
// 腳位定義（FL 已接：馬達 6/7 + 編碼器 14/15）
// ============================================================================
namespace Pin {
// 硬體 UART0 — 與 Hub 通訊（SlaveComms 內部處理 TX=GP0, RX=GP1）

// PWM + DIR for 4 wheels（依實測驗證結果：FR 6/7, FL 9/8,
// BR 10/11, BL 12/13）
constexpr uint MOTOR_FR_PWM = 6;
constexpr uint MOTOR_FR_DIR = 7;
constexpr uint MOTOR_FL_PWM = 9;
constexpr uint MOTOR_FL_DIR = 8;
constexpr uint MOTOR_BR_PWM = 10;
constexpr uint MOTOR_BR_DIR = 11;
constexpr uint MOTOR_BL_PWM = 12;
constexpr uint MOTOR_BL_DIR = 13;

// Encoder base pins（base & base+1，A=base）；依實測驗證結果：
// FR=14,15 FL=16,17 BR=20,21 BL=18,19
constexpr uint MOTOR_FR_ENC = 14; // FR: A=14, B=15
constexpr uint MOTOR_FL_ENC = 16; // FL: A=16, B=17
constexpr uint MOTOR_BR_ENC = 20; // BR: A=20, B=21
constexpr uint MOTOR_BL_ENC = 18; // BL: A=18, B=19
} // namespace Pin

// ============================================================================
// 參數
// ============================================================================
namespace MoveCfg {
// 硬體 PWM wrap（解析度）— clkdiv=6.25, wrap=1000 → ~20kHz
constexpr uint16_t PWM_WRAP = 1000;
// 麥克納姆：車身半長/半寬/輪半徑 (m)
constexpr float HALF_LENGTH = 0.25f;
constexpr float HALF_WIDTH = 0.25f;
constexpr float WHEEL_RADIUS = 0.05f;

// 編碼器
constexpr uint16_t ENC_LINE_PPR = 443;                    // 實測線數
constexpr uint16_t ENC_COUNTS_PER_REV = ENC_LINE_PPR * 4; // 4X → 1772

// 閉迴路 PID 出力上限（duty%）。
//   採保守 30%（與早期調参/歷史一致）：「命令+ → 馬達最多 30%」。
//   安全煞車，避免 PID 發散時全出力衝撞。若確認穩定後要更大出力再放寬。
constexpr float PID_MAX_DUTY = 100.0f;
// 積分放寬「風扇」餘裕（duty%）：積分上下限 = |ff*target| + 此值。
//   讓積分只補足命令前饋以下的小誤差，避免低速時積分風扇到 ±100
//   造成爆衝、也避免歸零時一次性大幅跳變。若慢速摩擦力需更多出力再加大。
constexpr float WINDUP_MARGIN_DUTY = 6.0f;
// 速度量測一階低通時間常數 (s)
constexpr float VEL_LPF_TAU = 0.010f;

// 停車指令判定閾值 (RPS)：目標輪轉速低於此值視為「停」，
//   強制清空 PID 積分，避免殘留積分造成零指令下馬達緩慢蠕動。
constexpr float StopCmdEpsRps = 0.005f;

// ---- 車身 S 曲線速度剖面（SCurveRamp）參數 ----
//   單位：車身速度為 RPS，加速度 RPS/s，Jerk RPS/s²。
//   目標行為：
//     起步（從 0）→ 慢段小坡克服靜摩擦 → 中段加速 → 平滑到位
//     減速（往 0）→ 快速但 Jerk 限幅、不瞬停
//   這些值需在實測車上調（依車重/摩擦/PID）；以下為初始合理值。
constexpr float SCURVE_ACCEL = 4.0f;  // 加速上限 (RPS/s) — 平緩加速
constexpr float SCURVE_DECEL = 12.0f; // 減速上限 (RPS/s) — 快速減速(3×加速)
constexpr float SCURVE_JERK = 60.0f;  // Jerk 上限 (RPS/s²) — S 形圓角
constexpr float SCURVE_START_SLOW_VEL = 0.20f; // 起步慢段速度窗 (RPS)
constexpr float SCURVE_START_ACCEL =
    1.5f; // 起步加速上限 (RPS/s) — 慢提克服靜摩擦
constexpr float SCURVE_SETTLE_TOL = 0.010f; // 到位判定 (RPS)
} // namespace MoveCfg

// ============================================================================
// 四輪閉迴路 PID 參數（由實測綁 PIO 架構依序調出；單位 duty% per
// RPS）
//   輪序 = [FL, FR, BL, BR]
// ============================================================================
struct WheelPidCfg {
    float kP, kI, kD, ff;
};
static const WheelPidCfg kPidCfg[4] = {
    {2.40f, 10.4f, 0.0f, 15.2f}, // FL
    {2.11f, 8.80f, 0.0f, 15.0f}, // FR
    {2.09f, 8.70f, 0.0f, 14.8f}, // BL
    {2.40f, 10.4f, 0.0f, 15.2f}, // BR
};

// 每輪極性：依實測驗證（FR/BR 反轉），輪序 = [FL,
// FR, BL, BR]。
//   cmd.vx>0（往前）→ 四輪 targetRps 全正；FR/BR 經 -1 反相馬達（鏡像）→
//   物理往前。
static const int8_t kWheelPolarity[4] = {1, -1, 1, -1};
// 編碼器讀取極性：FR/BR 因鏡像安裝在讀取端反轉 → 「馬達往前」四顆同號。
//   與 kWheelPolarity 相同，形成自洽閉環（命令 + → 馬達往前 → 回授 +）。
static const int8_t kEncReadPolarity[4] = {1, -1, 1, -1};

// 車身兩條平移軸（vx = 左右 strafe、vy = 前後 forward）各自獨立方向開關。
//   依實測車身方向微調，每條軸獨立 +1/-1，互不干擾 ω（旋轉）。
//   ⚠ 四輪閉迴路為逐輪自洽伺服，且方向與 ω 皆經由同一組輪速組合產生，
//     因此只能「一次翻一條軸」在車上實測確認，勿兩條同時猜。
static const int8_t kAxisStrafePolarity = 1;
static const int8_t kAxisForwardPolarity = -1;

// ============================================================================
// 車身 S 曲線速度剖面（jerk-limited，非對稱加/減速）
//   每車身軸（vx / vy / omega）一套，單位皆 RPS（加速度 RPS/s）。
//   作用於原始指令 → 平滑後指令，再餵給運動學/PID。
//   目標：
//     起步（從 0）→ 慢段小坡克服靜摩擦 → 中段加速 → 平滑到位
//     減速（往 0）→ 快速但 Jerk 限幅、不瞬停
//   穩態追蹤時 y==target（無誤差、無積分風扇）；僅在目標改變時作用。
// ============================================================================
struct SCurveRamp {
    float y = 0.f;     // 平滑後指令輸出（此軸）
    float accel = 0.f; // 目前加速度（RPS/s）

    // 更新並回傳平滑後輸出（一行呼叫）
    float updatePw(float target, float dt) {
        update(target, dt);
        return y;
    }

    // 每控制週期呼叫，把 y 往 target 平滑推進
    void update(float target, float dt) {
        float err = target - y;
        if(fabsf(err) < MoveCfg::SCURVE_SETTLE_TOL) {
            y = target;
            accel = 0.f;
            return;
        }
        // 判斷加/減速：目標比目前更靠近 0 → 減速（快速，且維持方向）
        const bool decel = (y != 0.f) && ((err > 0.f) != (y > 0.f));
        float cap = decel ? MoveCfg::SCURVE_DECEL : MoveCfg::SCURVE_ACCEL;
        // 起步慢段：從 0 出發且速度尚小 → 限加速度以小斜率起步克服靜摩擦
        if(!decel && fabsf(y) < MoveCfg::SCURVE_START_SLOW_VEL)
            cap = (cap < MoveCfg::SCURVE_START_ACCEL)
                      ? cap
                      : MoveCfg::SCURVE_START_ACCEL;

        // Jerk 限幅：加速度以 kJerk 斜率往目標 cap 逼近 → S 形圓角
        const float aim = (err > 0.f) ? cap : -cap;
        const float maxDa = MoveCfg::SCURVE_JERK * dt;
        float da = aim - accel;
        if(da > maxDa)
            da = maxDa;
        if(da < -maxDa)
            da = -maxDa;
        accel += da;

        y += accel * dt;

        // 防止越過目標（overshoot）：越過即鎖定在 target
        if((y - target) * err < 0.f)
            y = target;
    }
};
static SCurveRamp rampVx, rampVy, rampOmega; // 車身三軸各一套

// ============================================================================
// 四輪狀態
// ============================================================================
struct WheelState {
    uint pwmPin;
    uint dirPin;
    uint encPin;             // encoder base pin
    float targetDuty = 0.0f; // 目標 duty%（未直接使用，PID 直接算 out）
    float outDuty = 0.0f;    // 實際輸出 duty%
    int32_t encCount = 0;    // 編碼器計數（Host 端累計）
    float measRps = 0.0f;    // 低通後量測轉速（RPS）
    float integral = 0.0f;   // PI 積分項
    float prevMeas = 0.0f;   // 微分用（kD=0 未用）
    bool hasPrev = false;
};
static WheelState wheels[4];
static uint64_t lastTickUs = 0; // Core 1 控制週期時間戳

// ============================================================================
// 四輪編碼器 — PIO 讀取（PIO0 4 SM，無 GPIO 中斷）
//   使用實測驗證成功之架構：
//     - pio_gpio_init 綁腳（RP2350 必需）
//     - `in pins,2` 用 shift_right=false（left shift，base pin 落 bit0）
//     - clkdiv=5000 降採樣 ~25kHz 濾機械回彈
//     - Core 1 控制週期 drain FIFO + Gray 表累計，並套 kEncReadPolarity
//     統一方向
// ============================================================================
static PIO g_pio = pio0;
static uint g_pioOffset = 0;

// host-side 有向累計計數 + 上一相位（已套 kEncReadPolarity 統一符號）
static volatile int32_t enc[4];
static uint8_t encPrev[4];

// 正交解碼表（Gray）。索引 = ab 相位，非相鄰跳變 = 0。
static const int8_t kQuadTable[4][4] = {
    {0, +1, -1, 0},
    {-1, 0, 0, +1},
    {+1, 0, 0, -1},
    {0, -1, +1, 0},
};

// drain FIFO：把 PIO 推進來的相位取樣，逐一累計 to enc[]（套極性）。
//   每輪最多 MAX_DRAIN 筆即停，避免無限追尾卡死。Core 1 控制週期呼叫。
static void drainEncoders() {
    constexpr int MAX_DRAIN = 64;
    for(int i = 0; i < 4; i++) {
        int n = 0;
        while(!pio_sm_is_rx_fifo_empty(g_pio, i) && n < MAX_DRAIN) {
            const uint32_t cur = pio_sm_get(g_pio, i) & 0x3;
            enc[i] += kEncReadPolarity[i] * kQuadTable[encPrev[i]][cur];
            encPrev[i] = static_cast<uint8_t>(cur);
            n++;
        }
    }
}

static void encoderInit() {
    g_pioOffset = pio_add_program(g_pio, &quad_phase_program);
    // 依 wheels 順序 (FL,FR,BL,BR) 初始化 enc[]：與 wheels[i]/運動學對應
    constexpr uint encPin[4] = {Pin::MOTOR_FL_ENC, Pin::MOTOR_FR_ENC,
                                Pin::MOTOR_BL_ENC, Pin::MOTOR_BR_ENC};
    for(int i = 0; i < 4; i++) {
        const uint aPin = encPin[i];
        const uint bPin = aPin + 1;
        gpio_init(aPin);
        gpio_init(bPin);
        gpio_set_dir(aPin, GPIO_IN);
        gpio_set_dir(bPin, GPIO_IN);
        gpio_pull_up(aPin);
        gpio_pull_up(bPin);
        pio_gpio_init(g_pio, aPin);
        pio_gpio_init(g_pio, bPin);

        pio_sm_config c = quad_phase_program_get_default_config(g_pioOffset);
        sm_config_set_in_pins(&c, aPin);              // base pin = A
        sm_config_set_in_shift(&c, false, false, 32); // left shift（關鍵）
        sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
        sm_config_set_clkdiv(&c, 5000.0f); // ~25kHz，濾回彈

        pio_sm_init(g_pio, i, g_pioOffset, &c);
        pio_sm_clear_fifos(g_pio, i);
        pio_sm_set_enabled(g_pio, i, true);

        enc[i] = 0;
        const uint8_t a = gpio_get(aPin) ? 1u : 0u;
        const uint8_t b = gpio_get(bPin) ? 1u : 0u;
        encPrev[i] = static_cast<uint8_t>(a | (b << 1)); // 與 PIO cur 一致
    }
}

// ============================================================================
// 前向宣告
// ============================================================================
static void onMoveCommand(const ChassisSnapshot& cmd);
static void onMoveEstop();

// ============================================================================
// SlaveApp 回呼 — SlaveComms 在 Core 1 呼叫
// ============================================================================
static SlaveApp moveApp = {
    .targetID = TargetID::MOVE,
    .onCommand = onMoveCommand,
    .onEstop = onMoveEstop,
};

static SlaveComms slave(&moveApp);

// ============================================================================
// 命令除錯快照 — Core 1 寫入，Core 0 印出（遵循專案規範）
// ============================================================================
static volatile float dbg_vx = 0.0f;
static volatile float dbg_vy = 0.0f;
static volatile float dbg_omega = 0.0f;
static volatile bool dbg_estop = false;
static volatile uint32_t dbg_cmdCount = 0; // Core 1 實際執行 onCommand 的次數
static volatile float dbg_measRps = 0.0f;  // FL(輪0) 量測轉速
static volatile float dbg_outDuty0 = 0.0f; // FL(輪0) 實際輸出 duty%

// ============================================================================
// 硬體 PWM 輸出 + DIR（對齊 motor.h 行為）
//   - duty>=0 → DIR=LOW(正轉)；duty<0 → DIR=HIGH(反轉)，輸出 |duty|
//   - 輸入為 duty%（-100~100）；極性已在呼叫端乘以 kWheelPolarity[]
// ============================================================================
static void wheelSetPwm(uint pwmPin, uint dirPin, float dutyPercent) {
    bool fwd = (dutyPercent >= 0.0f);
    gpio_put(dirPin, fwd ? 0 : 1); // LOW=正轉, HIGH=反轉
    float a = fabsf(dutyPercent);
    if(a > 100.0f)
        a = 100.0f;
    uint16_t level = static_cast<uint16_t>(a / 100.0f * MoveCfg::PWM_WRAP);
    pwm_set_gpio_level(pwmPin, level);
}

static void initWheelPwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 6.25f); // 125MHz / (1000+1) / 6.25 ≈ 20kHz
    pwm_config_set_clkdiv_mode(&cfg, PWM_DIV_FREE_RUNNING);
    pwm_config_set_wrap(&cfg, MoveCfg::PWM_WRAP);
    pwm_init(slice, &cfg, false);
    pwm_set_gpio_level(gpio, 0);
    pwm_set_enabled(slice, true);
}

// ============================================================================
// onMoveCommand — Core 1 收到命令時執行一次控制週期
//   四輪皆速度閉迴路 PID：編碼器 RPS 回授 → ff + PI（抗飽和）→ duty%
// ============================================================================
static void onMoveCommand(const ChassisSnapshot& cmd) {
    dbg_cmdCount++;
    dbg_vx = cmd.vx;
    dbg_vy = cmd.vy;
    dbg_omega = cmd.omega;
    dbg_estop = false;

    // 先消化 PIO 編碼器 FIFO（Core 1 控制週期內）
    drainEncoders();

    // 控制週期 dt（跨幀時間）
    uint64_t nowUs = time_us_64();
    float dt = static_cast<float>(nowUs - lastTickUs) * 1e-6f;
    if(dt < 1e-4f)
        dt = 1e-4f;
    if(dt > 0.05f)
        dt = 0.05f;
    lastTickUs = nowUs;

    // ---- 車身 S 曲線速度剖面 ----
    //   平滑原始指令 → 平滑後指令，再餵給運動學/PID。
    //   起步（從 0）慢提速克服靜摩擦、減速快速但不瞬停。
    const float rVx = rampVx.updatePw(cmd.vx, dt);
    const float rVy = rampVy.updatePw(cmd.vy, dt);
    const float rOmega = rampOmega.updatePw(cmd.omega, dt);

    // 車身速度 (RPS) → 各輪目標線速度 (m/s)
    //   rVx>0（往前）→ lin[0..3] 全正；FR/BR 再由輸出極性反相（鏡像）。
    float wx = kAxisStrafePolarity * rVx * TWO_PI;  // rad/s（左右軸方向開關）
    float wy = kAxisForwardPolarity * rVy * TWO_PI; // rad/s（前後軸方向開關）
    float K = MoveCfg::HALF_LENGTH + MoveCfg::HALF_WIDTH;
    float lin[4];
    lin[0] = (wx - wy - K * rOmega) * MoveCfg::WHEEL_RADIUS; // FL
    lin[1] = (wx + wy + K * rOmega) * MoveCfg::WHEEL_RADIUS; // FR
    lin[2] = (wx + wy - K * rOmega) * MoveCfg::WHEEL_RADIUS; // BL
    lin[3] = (wx - wy + K * rOmega) * MoveCfg::WHEEL_RADIUS; // BR

    // 目標輪轉速 targetRps[i] = lin / (2π*r)，單位 rev/s (RPS)
    float targetRps[4];
    for(int i = 0; i < 4; i++) {
        targetRps[i] = lin[i] / (TWO_PI * MoveCfg::WHEEL_RADIUS);
    }

    // ---- 四輪閉迴路 PID ----
    const float alpha = dt / (MoveCfg::VEL_LPF_TAU + dt);
    for(int i = 0; i < 4; i++) {
        WheelState& w = wheels[i];
        const int32_t c = enc[i]; // 已套 kEncReadPolarity（統一方向）
        const int32_t d = c - w.encCount;
        w.encCount = c;
        // 低通量測轉速（含方向；enc 已統一符號 → 正目標對正量測）
        float rawRps = static_cast<float>(d) /
                       static_cast<float>(MoveCfg::ENC_COUNTS_PER_REV) / dt;
        w.measRps += alpha * (rawRps - w.measRps);

        const float err = targetRps[i] - w.measRps;

        // 「停車指令」：目標轉速≈0 時強制清空積分。
        //   先前移動時積分會蓄積穩態偏置（蓄積越大維持速度的 bias），
        //   若不清空，煞車後輸出 ≈ 殘留積分 → 馬達在零指令下仍緩慢蠕動。
        //   清空後於減速過程僅剩 P 項（-kP*meas）主動煞至 0 再穩住。
        if(fabsf(targetRps[i]) < MoveCfg::StopCmdEpsRps)
            w.integral = 0.0f;

        // ---- PI 控制器（含抗風扇 + 低速精細化）----
        const WheelPidCfg& pid = kPidCfg[i];
        const float ff = pid.ff * targetRps[i]; // 前饋出力 (duty%)
        const float p = pid.kP * err;

        // 積分上下限隨指令縮放：只讓積分補足「命令前饋出力以下」的誤差。
        //   低速指令 → 積分維持很小 → 能精細控制、且歸零時不會 80→0 大跳變。
        //   （+WINDUP_MARGIN_DUTY 為摩擦/動態餘裕）
        float iLimit = fabsf(ff) + MoveCfg::WINDUP_MARGIN_DUTY;
        if(iLimit > MoveCfg::PID_MAX_DUTY)
            iLimit = MoveCfg::PID_MAX_DUTY;

        // 抗風扇：積分已達縮放上限且誤差同向（繼續累積只會加劇飽和）→ 暫停
        const bool windup = (w.integral >= iLimit && err > 0.f) ||
                            (w.integral <= -iLimit && err < 0.f);
        if(!windup)
            w.integral += pid.kI * err * dt;
        if(w.integral > iLimit)
            w.integral = iLimit;
        if(w.integral < -iLimit)
            w.integral = -iLimit;

        float dterm = 0.0f;
        if(w.hasPrev) {
            dterm = -pid.kD * (w.measRps - w.prevMeas) / dt;
        }
        w.prevMeas = w.measRps;
        w.hasPrev = true;

        float out = p + w.integral + dterm + ff;
        if(out > MoveCfg::PID_MAX_DUTY)
            out = MoveCfg::PID_MAX_DUTY;
        if(out < -MoveCfg::PID_MAX_DUTY)
            out = -MoveCfg::PID_MAX_DUTY;

        w.outDuty = out;
        wheelSetPwm(w.pwmPin, w.dirPin, out * kWheelPolarity[i]);
    }
    dbg_measRps = wheels[0].measRps;
    dbg_outDuty0 = wheels[0].outDuty;

    // ---- 遙測（四輪 current/target/輸出；由 Core 0 update() 定時送出）----
    MoveTelemetry tm = {};
    for(int i = 0; i < 4; i++) {
        tm.currentRps[i] = wheels[i].measRps;
        tm.targetRps[i] = targetRps[i];
        tm.pwmOutput[i] = wheels[i].outDuty;
    }
    tm.cmdVx = rVx;
    tm.cmdVy = rVy;
    tm.cmdOmega = rOmega;
    tm.mode = (uint8_t)RobotMode::AUTO;
    tm.estopFlag = 0;
    tm.timestamp = (uint32_t)(nowUs & 0xFFFFFFFFULL);
    slave.prepareTelemetry(tm);
}

// ============================================================================
// onMoveEstop — 立即歸零所有 PWM，並清 PID 積分
// ============================================================================
static void onMoveEstop() {
    for(int i = 0; i < 4; i++) {
        wheels[i].targetDuty = 0.0f;
        wheels[i].outDuty = 0.0f;
        wheels[i].integral = 0.0f;
        wheels[i].hasPrev = false;
        wheelSetPwm(wheels[i].pwmPin, wheels[i].dirPin, 0.0f);
    }
    dbg_estop = true;
}

// ============================================================================
// Core 0
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("[MOVE] FL 閉迴路 PID + 其他開迴路（等待 Hub 命令）");

    slave.begin();

    constexpr uint pwmPins[4] = {Pin::MOTOR_FL_PWM, Pin::MOTOR_FR_PWM,
                                 Pin::MOTOR_BL_PWM, Pin::MOTOR_BR_PWM};
    constexpr uint dirPins[4] = {Pin::MOTOR_FL_DIR, Pin::MOTOR_FR_DIR,
                                 Pin::MOTOR_BL_DIR, Pin::MOTOR_BR_DIR};
    constexpr uint encPins[4] = {Pin::MOTOR_FL_ENC, Pin::MOTOR_FR_ENC,
                                 Pin::MOTOR_BL_ENC, Pin::MOTOR_BR_ENC};

    for(int i = 0; i < 4; i++) {
        initWheelPwm(pwmPins[i]);
        wheels[i].pwmPin = pwmPins[i];
        wheels[i].dirPin = dirPins[i];
        wheels[i].encPin = encPins[i];
        wheels[i].targetDuty = 0.0f;
        wheels[i].outDuty = 0.0f;

        gpio_init(dirPins[i]);
        gpio_set_dir(dirPins[i], GPIO_OUT);
        gpio_put(dirPins[i], 1);
    }
}

void loop() {
    slave.update();

    static uint32_t lastPrint = 0;
    uint32_t nowCfg = millis();
    if(nowCfg - lastPrint >= 100) {
        lastPrint = nowCfg;
        uint32_t since = slave.msSinceLastValidCmd();
        uint32_t cnt = slave.validCmdCount();
        Serial.printf("[MOVE] sinceCmd=%ums cnt=%u onCmd=%u\n", since, cnt,
                      dbg_cmdCount);
        if(dbg_estop) {
            Serial.println("[MOVE] ** E-STOP active **");
        } else {
            Serial.printf("[MOVE] vx=%+.2f vy=%+.2f omega=%+.2f | "
                          "FL meas=%+.2fRPS out=%+.0f%%\n",
                          dbg_vx, dbg_vy, dbg_omega, dbg_measRps, dbg_outDuty0);
            Serial.printf("       FR meas=%+.2f out=%+.0f | "
                          "BL meas=%+.2f out=%+.0f | "
                          "BR meas=%+.2f out=%+.0f\n",
                          wheels[1].measRps, wheels[1].outDuty,
                          wheels[2].measRps, wheels[2].outDuty,
                          wheels[3].measRps, wheels[3].outDuty);
        }
    }
}

// ============================================================================
// Core 1 — 自旋等待 SlaveComms 派發命令，執行四輪 PID 控制
// ============================================================================
void setup1() {
    // PIO 編碼器初始化（Core 1 啟用；PIO SM 為硬體計數，不綁核心）
    encoderInit();
}

void loop1() { slave.runCore1(); }
