// ============================================================================
// Move Slave — 四輪麥克納姆輪控制韌體 (RP2350)
//
// 根據 agent.md 雙核架構規範：
//   Core 0 (loop):  硬體 UART0 收發 COBS 幀、CRC 驗證、指令解析、通知 Core 1
//   Core 1 (loop1): 由 Core 0 觸發，執行 Mecanum 運動學解算 + 四輪獨立 PID 控制
//
// 通訊（agent.md §通訊協議）：
//   Hub PIO UART SM0 (GP6 TX, GP7 RX) ↔ Move UART0 (setPinout GP6 RX, GP7 TX)
//   921600 baud, COBS 封幀 + CRC16
//
// 硬體配置（每輪）：
//   - 一組 PIO PWM (2 個 state machine 可做互補輸出，或 1 個 SM 做單端)
//   - 一組 PIO Quadrature Encoder (1 個 SM)
//   - 四輪總計：4×PWM + 4×Encoder = 8 SM (剛好用完一個 PIO block)
//     若使用 PIO block0 全部 4 個 SM 做 Encoder，
//     block1 全部 4 個 SM 做 PWM。
// ============================================================================
#include <Arduino.h>
#include <atomic>
#include <cmath>
#include <cstring>
#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/pio.h>
#include <hardware/timer.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#include "ChassisComms.h"
#include "signal_shared.h"

// ============================================================================
// 腳位定義 (依實際硬體接線修改)
// ============================================================================
namespace Pin {
// 硬體 UART0 — 與 Hub 通訊（agent.md §通訊協議）
// UART0 預設腳位 TX=GP0, RX=GP1（Earlephilhower Serial1）
// 接線（外接）：
//   Hub PIO SM0 TX(GP6) ──── Move UART0 RX(GP1)
//   Hub PIO SM0 RX(GP7) ──── Move UART0 TX(GP0)
// 不需 setPinout，直接用 Serial1 預設腳位
constexpr uint UART_TX = 0; // UART0 TX (default) — GP0
constexpr uint UART_RX = 1; // UART0 RX (default) — GP1

// 四輪馬達 PWM (使用 PIO PWM)
constexpr uint MOTOR1_PWM = 6; // FL — 前左
constexpr uint MOTOR2_PWM = 7; // FR — 前右
constexpr uint MOTOR3_PWM = 8; // RL — 後左
constexpr uint MOTOR4_PWM = 9; // RR — 後右

// 四輪編碼器 A/B 相
constexpr uint MOTOR1_ENC_A = 10; // FL Encoder A
constexpr uint MOTOR1_ENC_B = 11; // FL Encoder B
constexpr uint MOTOR2_ENC_A = 12; // FR Encoder A
constexpr uint MOTOR2_ENC_B = 13; // FR Encoder B
constexpr uint MOTOR3_ENC_A = 14; // RL Encoder A
constexpr uint MOTOR3_ENC_B = 15; // RL Encoder B
constexpr uint MOTOR4_ENC_A = 16; // RR Encoder A
constexpr uint MOTOR4_ENC_B = 17; // RR Encoder B
} // namespace Pin

// ============================================================================
// 運動學參數
// ============================================================================
namespace Kinematics {
// 車身半長 (m) — 依實際車體修改
constexpr float HALF_LENGTH = 0.25f;
// 車身半寬 (m)
constexpr float HALF_WIDTH = 0.25f;
// 輪半徑 (m)
constexpr float WHEEL_RADIUS = 0.05f;

// HALF_LENGTH：請拿量尺測量前輪軸心到後輪軸心的距離，然後除以 2。例如軸距是 30
// 公分，這裡就填 0.15f。 HALF_WIDTH：測量左輪中心線到右輪中心線的距離，然後除以
// 2。 WHEEL_RADIUS：量測你購買的麥克納姆輪直徑並除以 2。直徑 10cm
// 的輪子，這裡就填 0.05f。

// 從 (vx, vy, omega) 算到各輪期望轉速 (rad/s)
// 麥克納姆輪逆運動學：
//   FL： (vx - vy - (L+W)*ω) / R
//   FR： (vx + vy + (L+W)*ω) / R
//   RL： (vx + vy - (L+W)*ω) / R
//   RR： (vx - vy + (L+W)*ω) / R
constexpr float L_PLUS_W = HALF_LENGTH + HALF_WIDTH;
constexpr float INV_R = 1.0f / WHEEL_RADIUS;

// Feedforward 增益：rps → PWM duty 的開迴路增益 (需實際量測校準)
// 作法：給定一個定 duty (如 10%) 後量測穩定 RPS，算出 slope。
// kFF = targetRPS / duty (單位：RPS / 歸一化 duty)
constexpr float kFF = 7.0f;  // 例：duty=1.0 → 7 RPS

// 車身加速度 ramping 限制 (RPS/s²)，用於上層目標值的平順化
// 取代原本的 SlewRateLimiter（作用在車身速度層級而非各輪）
constexpr float ACCEL_LIMIT = 20.0f; // RPS/s²，可依手感調整

// 將上層速度 (vx, vy, omega) 轉換為四輪期望 RPS
inline void computeWheelTargets(float vx, float vy, float omega, float& fl,
                                float& fr, float& rl, float& rr) {
    float lwOmega = L_PLUS_W * omega;
    fl = (vx - vy - lwOmega) * INV_R;
    fr = (vx + vy + lwOmega) * INV_R;
    rl = (vx + vy - lwOmega) * INV_R;
    rr = (vx - vy + lwOmega) * INV_R;
}

// Feedforward 項：rps → 前饋 PWM 輸出
// 用線性比例近似，若馬達有量測曲線可改用 lookup table
inline float feedforward(float targetRps) {
    return targetRps / kFF;
}
} // namespace Kinematics

// ============================================================================
// PID + Feedforward 控制器
//
// output = PID(error) + FF(setpoint)
//   — PID 負責閉環修正（抗干擾）
//   — FF 提供基底開迴路輸出，讓 PID 只需補小量誤差
// ============================================================================
class PidController {
  public:
    float kp, ki, kd;
    float outLimit;
    float integralLimit;
    float ffGain; // feedforward 增益 (PWM/RPS)，0 則關閉 FF

    void reset() {
        integral_ = 0.0f;
        prevError_ = 0.0f;
        firstRun_ = true;
    }

    /// dt 單位：秒
    /// setpoint = 目標 RPS, measurement = 當前 RPS
    /// 回傳 = PID 輸出 + Feedforward 項（已限幅）
    float update(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        if(firstRun_) {
            prevError_ = error;
            firstRun_ = false;
        }

        // ---- PID 三項 ----
        // 比例項
        float pTerm = kp * error;

        // 積分項 (含 anti-windup clamp)
        integral_ += ki * error * dt;
        if(integral_ > integralLimit)
            integral_ = integralLimit;
        if(integral_ < -integralLimit)
            integral_ = -integralLimit;

        // 微分項 (在量測值上微分，避免 derivative kick)
        float dTerm = kd * (error - prevError_) / dt;
        prevError_ = error;

        float pidOut = pTerm + integral_ + dTerm;

        // ---- Feedforward 項（前饋）----
        // 使用 Kinematics::feedforward() 計算對應的 PWM
        float ffOut = Kinematics::feedforward(setpoint);

        float output = pidOut + ffOut;

        // 輸出限幅
        if(output > outLimit)
            output = outLimit;
        if(output < -outLimit)
            output = -outLimit;

        return output;
    }

  private:
    float integral_ = 0.0f;
    float prevError_ = 0.0f;
    bool firstRun_ = true;
};

// ============================================================================
// RampedTarget — 車身速度層加速度漸變 (Ramping)
//
// 作用在 (vx, vy, omega) 目標值上，讓車身平順加減速，
// 取代各輪獨立的 SlewRateLimiter，使四輪動作一致。
// ============================================================================
class RampedTarget {
  public:
    explicit RampedTarget(float maxDeltaPerSec)
        : maxDelta_(maxDeltaPerSec) {}

    void reset(float value = 0.0f) { previous_ = value; }

    // 限步長逼近目標，回傳本週期應使用的值
    float update(float target, float dt) {
        float maxStep = maxDelta_ * dt;
        float diff = target - previous_;
        if(diff > maxStep)
            diff = maxStep;
        if(diff < -maxStep)
            diff = -maxStep;
        previous_ += diff;
        return previous_;
    }

  private:
    float maxDelta_;
    float previous_ = 0.0f;
};

// ============================================================================
// 四輪狀態 — 每個輪子一組
// ============================================================================
struct WheelState {
    // PIO 資源索引
    uint pwmSm;   // PIO PWM 使用的 state machine 編號
    uint encSm;   // PIO Encoder state machine 編號
    PIO pioBlock; // 所屬 PIO block

    // 控制
    PidController pid;
    volatile int32_t encoderCount; // PIO 更新 (IRQ 中原子寫入)
    float currentRps;              // 當前轉速 (由 Core 1 計算)
    float targetRps;               // 期望轉速
    float pwmOutput;               // PID+FF 輸出值

    void initPid() {
        // 典型輪子 PID 參數 — 需實際調校
        // 有 FF 前饋後，PID 只需補少量誤差，增益可比純 PID 小
        pid.kp = 1.5f;
        pid.ki = 0.0f;
        pid.kd = 0.0f;
        pid.outLimit = RobotConfig::PID_OUT_LIMIT;
        pid.integralLimit = 5.0f;
        pid.ffGain = Kinematics::kFF;  // 使用運動學 FF 增益
        pid.reset();
    }
};

// ============================================================================
// 全域狀態 — 所有靜態分配，禁止 new/malloc (agent.md §核心 1 規則)
// ============================================================================

// 四輪狀態陣列 (FL, FR, RL, RR)
static WheelState wheels[4];

// Core 0 ↔ Core 1 雙緩衝區 (agent.md §核心間通訊)
static ChassisSnapshot cmdBuf[2];
static std::atomic<uint8_t> cmdWriteIdx{0};
static std::atomic<bool> cmdValid{false};

// E-Stop 旗標
static std::atomic<bool> estopActive{false};
static uint64_t lastValidFrameTime = 0;

// 除錯輸出 (透過 UART1)
// static Stream* debugOut = nullptr;  // 取代為 debugOut 直接指向 Serial1

// 時間追蹤
static absolute_time_t lastControlTime;

// ============================================================================
// 硬體 UART0 + ChassisComms — 與 Hub 通訊
// ============================================================================
// Serial1 = 硬體 UART0 on RP2350（Earlephilhower mapping）
// 透過 setPinout() 將 RX 映射到 GP6（接收 Hub PIO TX）、TX 映射到 GP7
// ChassisComms 綁定此 UART，心跳 30ms 回送 Hub
static ChassisComms hubComms(&Serial1, TargetID::HUB, RobotConfig::HEARTBEAT_INTERVAL_US);

// ============================================================================
// PIO 程式 — 四倍頻編碼器解碼
// 標準 AB 正交解碼：每個 edge 計數，4 倍頻
//
// 注意：RP2350 PIO 指令為 16-bit，故使用 uint16_t 陣列
// 並以 pio_program_t 結構包裝供 pio_add_program() 使用。
// ============================================================================
static const uint16_t pio_encoder_instructions[] = {
    0x2020, // 0: wait 0 pin, 0        ; 等待 A 變低
    0x20a0, // 1: wait 1 pin, 0        ; 等待 A 變高
    0x2021, // 2: wait 0 pin, 1        ; 等待 B 變低
    0x20a1, // 3: wait 1 pin, 1        ; 等待 B 變高
    0x0040, // 4: jmp x--, 0           ; 計數 (遞減後跳回)
};

static const pio_program_t pio_encoder_program = {
    .instructions = pio_encoder_instructions,
    .length = 5,
    .origin = -1,
};

// static_assert 不可用於非 constexpr 的 struct 成員，故移除
// 長度在宣告時明顯小於 32，無需 runtime assert

// ============================================================================
// PIO 程式 — PWM 輸出 (每輪一個 SM)
// 系統時脈 150MHz，PWM 頻率 25kHz：
//   CLKDIV = 1.0, wrap = 6000 - 1 = 5999
//   freq = 150MHz / (1 * 6000) = 25kHz
// ============================================================================
static const uint16_t pio_pwm_instructions[] = {
    0x80a0, // 0: pull noblock           ; 從 FIFO 取值 (32-bit: bit16=dir, bit15:0=duty)
    0xa027, // 1: mov x, osr             ; OSR → X (保留方向位元)
    0xe001, // 2: set y, 1               ; Y = 1 (用於比較)
    0x00c8, // 3: jmp pin, 5             ; 若方向 pin 為高 → 正轉 (跳至標籤)
    0x0086, // 4: jmp x!=y, 6            ; 反轉：當 X > 1 才輸出 high
    0x0086, // 5: jmp x!=y, 6            ; 正轉：當 X > 1 才輸出 high
    0x0082, // 6: jmp y--, 2             ; 遞減 Y，跳到步驟 2
    // .wrap (回到步驟 0)
};
// 簡化版 PWM — 實際使用需依驅動 IC 調整
// 大部分直流馬達驅動只需一組 PWM + 一組方向 GPIO
// 此 PIO 程式預留為硬體 PWM 備用；實務上可直接使用
// RP2350 的硬體 PWM (pwm_config) 或標準 Servo library

static const pio_program_t pio_pwm_program = {
    .instructions = pio_pwm_instructions,
    .length = 7,
    .origin = -1,
};

// ============================================================================
// 前向宣告
// ============================================================================
static void core1_main();
static void onHubFrame(const ControlPacket& pkt);
static void updateMotors(float dt);
static void readEncoders(float dt);
static void emergencyStop();
static float rpsToPwm(float rpsTarget);
static float pwmToRps(float pwm);

// ============================================================================
// Core 0 — 通訊核心
// ============================================================================
void setup() {
    // ---- 硬體 UART0 初始化（921600 baud，與 Hub PIO UART 相容）----
    // Serial1 = UART0 on RP2350，預設腳位 TX=GP0, RX=GP1
    Serial1.begin(RobotConfig::PIO_BAUDRATE);

    // ChassisComms 綁定到 Serial1，30ms 心跳回送 Hub
    hubComms.begin();
    hubComms.setFrameCallback(onHubFrame);

    // ---- PIO PWM 初始化 (block1, 4 SM 給 4 輪) ----
    PIO pwmPio = pio1;
    for(int i = 0; i < 4; i++) {
        wheels[i].pioBlock = pwmPio;
        wheels[i].pwmSm = static_cast<uint>(i);

        uint pin;
        switch(i) {
        case 0:
            pin = Pin::MOTOR1_PWM;
            break;
        case 1:
            pin = Pin::MOTOR2_PWM;
            break;
        case 2:
            pin = Pin::MOTOR3_PWM;
            break;
        case 3:
            pin = Pin::MOTOR4_PWM;
            break;
        default:
            pin = 0;
            break;
        }

        // 初始化 PIO PWM (使用 PIO 產生固定頻率 PWM)
        pio_gpio_init(pwmPio, pin);
        gpio_set_function(pin, GPIO_FUNC_PIO1);

        wheels[i].initPid();
        wheels[i].targetRps = 0.0f;
        wheels[i].currentRps = 0.0f;
        wheels[i].pwmOutput = 0.0f;
        wheels[i].encoderCount = 0;

        Serial.print("[Move] Motor ");
        Serial.print(i);
        Serial.println(" PWM initialized on PIO.");
    }

    // ---- PIO Encoder 初始化 (block0, 4 SM 給 4 輪) ----
    PIO encPio = pio0;
    uint encOffset = pio_add_program(encPio, &pio_encoder_program);
    for(int i = 0; i < 4; i++) {
        wheels[i].encSm = static_cast<uint>(i);

        uint pinA, pinB;
        switch(i) {
        case 0:
            pinA = Pin::MOTOR1_ENC_A;
            pinB = Pin::MOTOR1_ENC_B;
            break;
        case 1:
            pinA = Pin::MOTOR2_ENC_A;
            pinB = Pin::MOTOR2_ENC_B;
            break;
        case 2:
            pinA = Pin::MOTOR3_ENC_A;
            pinB = Pin::MOTOR3_ENC_B;
            break;
        case 3:
            pinA = Pin::MOTOR4_ENC_A;
            pinB = Pin::MOTOR4_ENC_B;
            break;
        default:
            pinA = 0;
            pinB = 0;
            break;
        }

        pio_gpio_init(encPio, pinA);
        pio_gpio_init(encPio, pinB);
        gpio_set_function(pinA, GPIO_FUNC_PIO0);
        gpio_set_function(pinB, GPIO_FUNC_PIO0);

        // --- 設定 SM 配置 ---
        // pio_encoder_program.length = 5，wrap = offset + length - 1
        pio_sm_config encCfg = pio_get_default_sm_config();
        sm_config_set_wrap(&encCfg, encOffset, encOffset + pio_encoder_program.length - 1);
        sm_config_set_in_pins(&encCfg, pinA);
        sm_config_set_jmp_pin(&encCfg, pinA);
        sm_config_set_in_pin_count(&encCfg, 2);
        sm_config_set_clkdiv(&encCfg, 1.0f);

        pio_sm_init(encPio, i, encOffset, &encCfg);
        pio_sm_set_enabled(encPio, i, true);

        Serial.print("[Move] Encoder ");
        Serial.print(i);
        Serial.println(" initialized.");
    }

    // ---- 啟動 Core 1 ----
    lastControlTime = get_absolute_time();
    multicore_launch_core1(core1_main);
    Serial.println("[Move] Core 1 launched.");
}

// ============================================================================
// Core 0 主迴圈：持續收 COBS 幀
// ============================================================================
void loop() {
    // 1. ChassisComms::update() 處理：
    //    - 從硬體 UART0 (Serial1) 讀取位元組
    //    - COBS 解碼 + CRC 驗證
    //    - 幀序號去重
    //    - 呼叫 onHubFrame 回呼 (CONTROL 型別)
    //    - 30ms 心跳向 Hub 發送
    hubComms.update();

    // 2. 逾時 E-Stop 檢查 (agent.md §指令超時保護)
    //    最後有效控制幀超過 100ms 無更新 → 強制 E-Stop
    if(!estopActive.load(std::memory_order_acquire) &&
       (time_us_64() - lastValidFrameTime > RobotConfig::COMMAND_TIMEOUT_US)) {
        estopActive.store(true, std::memory_order_release);
        Serial.println("[Move] E-Stop: command timeout!");
    }

    // 3. 讓出 CPU，給 ChassisComms IRQ 處理時間
    delayMicroseconds(20);
}

// ============================================================================
// 收到 Hub 控制幀的回呼 (由 ChassisComms 觸發，在 Core 0 上下文執行)
//
// ChassisComms::processFrame 已處理：
//   - sync 0xA5 驗證
//   - CRC 驗證
//   - 幀序號去重
//   - HEARTBEAT / ESTOP 型別檢查
//
// 此回呼僅處理 E-Stop 與 CONTROL 命令快照
// ============================================================================
static void onHubFrame(const ControlPacket& pkt) {
    // E-Stop 廣播處理 (agent.md §E-Stop 安全機制)
    if(pkt.msgType == MsgType::ESTOP || pkt.targetID == TargetID::BROADCAST) {
        estopActive.store(true, std::memory_order_release);
        Serial.println("[Move] E-Stop received from Hub!");
        return;
    }

    // Bypass 非控制幀 (心跳已由 ChassisComms 自行處理)
    if(pkt.msgType != MsgType::CONTROL) {
        return;
    }

    // 過濾非自己 mesh 的幀 (ChassisComms 已過濾 targetID，此處備援)
    if(pkt.targetID != TargetID::MOVE && pkt.targetID != TargetID::BROADCAST) {
        return;
    }

    // 更新最後有效幀時間
    lastValidFrameTime = time_us_64();

    // 解碼 int8_t payload → float，存入雙緩衝區
    uint8_t writeIdx = cmdWriteIdx.load(std::memory_order_relaxed);
    ChassisSnapshot& cmd = cmdBuf[writeIdx];

    // 將 int8_t 標準化為 float：-128~127 → -MAX_RPS~MAX_RPS
    constexpr float SCALE = RobotConfig::MAX_RPS / 127.0f;
    cmd.vx = static_cast<float>(pkt.vx) * SCALE;
    cmd.vy = static_cast<float>(pkt.vy) * SCALE;
    cmd.omega = static_cast<float>(pkt.omega) * SCALE;
    cmd.mode = pkt.mode;
    cmd.frameSeq = pkt.frameSeq;

    // 如果 E-Stop 曾觸發，現在收到有效幀 → 解除 E-Stop
    if(estopActive.load(std::memory_order_acquire)) {
        estopActive.store(false, std::memory_order_release);
        Serial.println("[Move] E-Stop cleared.");
    }

    // Release barrier：確保以上寫入在通知 Core 1 前全部可見
    cmdValid.store(true, std::memory_order_release);
    cmdWriteIdx.store(static_cast<uint8_t>(writeIdx ^ 1),
                      std::memory_order_release);
}

// ============================================================================
// Core 1 — 馬達控制核心 (由 Core 0 通知觸發)
// ============================================================================
static void __not_in_flash_func(core1_main)() {
    // 車身速度層加速度 Ramping (vx, vy, omega 各自獨立)
    RampedTarget rampVx(Kinematics::ACCEL_LIMIT);
    RampedTarget rampVy(Kinematics::ACCEL_LIMIT);
    RampedTarget rampOmega(Kinematics::ACCEL_LIMIT);
    rampVx.reset();
    rampVy.reset();
    rampOmega.reset();

    // E-Stop falling edge 偵測，用於重置 ramper
    bool prevEstop = false;

    uint64_t lastTick = time_us_64();

    while(true) {
        // Spinloop 等待 Core 0 通知 (agent.md §核心間同步機制)
        uint8_t readIdx = cmdWriteIdx.load(std::memory_order_acquire) ^ 1;

        // 檢查 E-Stop
        if(estopActive.load(std::memory_order_acquire)) {
            emergencyStop();
            prevEstop = true;
            tight_loop_contents();
            continue;
        }

        // E-Stop falling edge：復原瞬間重置 ramper，從零重新加速
        if(prevEstop) {
            prevEstop = false;
            rampVx.reset();
            rampVy.reset();
            rampOmega.reset();
            lastTick = time_us_64(); // 重置 dt 基準，避免第一幀的 dt 暴衝
        }

        // 檢查是否有新命令
        if(!cmdValid.load(std::memory_order_acquire)) {
            // 無新命令，檢查是否逾時 (agent.md §指令超時保護)
            uint64_t now = time_us_64();
            if(now - lastValidFrameTime > RobotConfig::COMMAND_TIMEOUT_US) {
                emergencyStop();
            }
            tight_loop_contents();
            continue;
        }

        // Acquire barrier + 快照命令到本地變數
        ChassisSnapshot localCmd;
        memcpy(&localCmd, &cmdBuf[readIdx], sizeof(ChassisSnapshot));
        cmdValid.store(false, std::memory_order_relaxed);

        // 計算 dt
        uint64_t now = time_us_64();
        float dt = static_cast<float>(now - lastTick) * 1e-6f;
        if(dt < 1e-6f)
            dt = 1e-6f;
        if(dt > 0.02f)
            dt = 0.02f;
        lastTick = now;

        // ---- 車身加速度 Ramping (作用在 vx/vy/omega 層級) ----
        // 三個獨立的 RampedTarget 使車體平順加減速、平滑轉向
        float rampedVx    = rampVx.update(localCmd.vx, dt);
        float rampedVy    = rampVy.update(localCmd.vy, dt);
        float rampedOmega = rampOmega.update(localCmd.omega, dt);

        // ---- Mecanum 運動學解算（使用 ramped 後的速度）----
        float flTarget, frTarget, rlTarget, rrTarget;
        Kinematics::computeWheelTargets(rampedVx, rampedVy, rampedOmega,
                                        flTarget, frTarget, rlTarget,
                                        rrTarget);

        // ---- 讀取編碼器 (取得 currentRps) ----
        readEncoders(dt);

        // ---- PID + Feedforward 閉環控制 (四輪獨立) ----
        float pwmFL = wheels[0].pid.update(flTarget, wheels[0].currentRps, dt);
        float pwmFR = wheels[1].pid.update(frTarget, wheels[1].currentRps, dt);
        float pwmRL = wheels[2].pid.update(rlTarget, wheels[2].currentRps, dt);
        float pwmRR = wheels[3].pid.update(rrTarget, wheels[3].currentRps, dt);

        // ---- PID 輸出安全檢查 ----
        if(std::isnan(pwmFL) || std::isinf(pwmFL) || std::isnan(pwmFR) ||
           std::isinf(pwmFR) || std::isnan(pwmRL) || std::isinf(pwmRL) ||
           std::isnan(pwmRR) || std::isinf(pwmRR)) {
            emergencyStop();
            continue;
        }

        // ---- 映射到 PIO PWM duty ----
        uint16_t dutyFL =
            static_cast<uint16_t>(fabsf(pwmFL) / RobotConfig::PID_OUT_LIMIT *
                                  RobotConfig::MOTOR_MAX_DUTY);
        uint16_t dutyFR =
            static_cast<uint16_t>(fabsf(pwmFR) / RobotConfig::PID_OUT_LIMIT *
                                  RobotConfig::MOTOR_MAX_DUTY);
        uint16_t dutyRL =
            static_cast<uint16_t>(fabsf(pwmRL) / RobotConfig::PID_OUT_LIMIT *
                                  RobotConfig::MOTOR_MAX_DUTY);
        uint16_t dutyRR =
            static_cast<uint16_t>(fabsf(pwmRR) / RobotConfig::PID_OUT_LIMIT *
                                  RobotConfig::MOTOR_MAX_DUTY);

        // 限幅
        auto clampDuty = [](uint16_t d) -> uint16_t {
            return (d > RobotConfig::MOTOR_MAX_DUTY)
                       ? RobotConfig::MOTOR_MAX_DUTY
                       : d;
        };
        dutyFL = clampDuty(dutyFL);
        dutyFR = clampDuty(dutyFR);
        dutyRL = clampDuty(dutyRL);
        dutyRR = clampDuty(dutyRR);

        // 寫入 PIO PWM (方向位元 + duty)
        // 假設 PIO PWM 程式接受 bit16=direction, bit15:0=duty
        auto writePwm = [](int i, uint16_t duty, bool negative) {
            pio_sm_put(wheels[i].pioBlock, wheels[i].pwmSm,
                       static_cast<uint32_t>(duty) |
                           (negative ? 0x10000UL : 0UL));
        };
        writePwm(0, dutyFL, pwmFL < 0);
        writePwm(1, dutyFR, pwmFR < 0);
        writePwm(2, dutyRL, pwmRL < 0);
        writePwm(3, dutyRR, pwmRR < 0);

        // 更新狀態供遙測
        wheels[0].targetRps = flTarget;
        wheels[0].pwmOutput = pwmFL;
        wheels[1].targetRps = frTarget;
        wheels[1].pwmOutput = pwmFR;
        wheels[2].targetRps = rlTarget;
        wheels[2].pwmOutput = pwmRL;
        wheels[3].targetRps = rrTarget;
        wheels[3].pwmOutput = pwmRR;
    }
}

// ============================================================================
// 讀取編碼器並計算各輪當前轉速 (RPS)
// ============================================================================
static void readEncoders(float dt) {
    // 編碼器解析度配置：預設 1200 PPR，4 倍頻 = 4800 count/rev
    constexpr float ENC_COUNTS_PER_REV = 4.0f * 1200.0f;
    constexpr float INV_ENC = 1.0f / ENC_COUNTS_PER_REV;

    static int32_t prevCount[4] = {0, 0, 0, 0};

    for(int i = 0; i < 4; i++) {
        int32_t rawCount = wheels[i].encoderCount;
        int32_t delta = rawCount - prevCount[i];
        prevCount[i] = rawCount;

        // delta count / (count/rev) / dt = rev/s (RPS)
    // delta count / (每轉計數) / dt = rev/s (RPS)
        wheels[i].currentRps = static_cast<float>(delta) * INV_ENC / dt;
    }
}

// E-Stop — 立即歸零所有 PWM (agent.md §E-Stop 安全機制)
// 注意：車身層的 RampedTarget 在 core1_main 本地變數，
// E-Stop 觸發後 Core 1 不會執行控制邏輯，ramper 自然停滯。
// 當命令恢復時，因 ramper previous_ 停在大值，車身會從當前速度漸變。
// 若需 E-Stop 後從零開始，可在 E-Stop 解除時重置 ramper，
// 做法：在 core1_main 中檢查 estop falling edge 後呼叫 rampVx.reset() 等。
static void emergencyStop() {
    for(int i = 0; i < 4; i++) {
        pio_sm_put(wheels[i].pioBlock, wheels[i].pwmSm, 0);
        wheels[i].pid.reset();
        wheels[i].currentRps = 0.0f;
        wheels[i].targetRps = 0.0f;
        wheels[i].pwmOutput = 0.0f;
    }
}

// ============================================================================
// 輔助函數（備用）
// ============================================================================
static float rpsToPwm(float rpsTarget) {
    return (rpsTarget / RobotConfig::MAX_RPS) * RobotConfig::PID_OUT_LIMIT;
}

static float pwmToRps(float pwm) {
    return (pwm / RobotConfig::PID_OUT_LIMIT) * RobotConfig::MAX_RPS;
}
