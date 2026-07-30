// ============================================================================
// motor_PPR_test — 四輪編碼器 PPR 量測工具 (RP2350)
//
// 用途：
//   用手慢慢轉動輪胎，觀察編碼器脈衝計數改變多少，
//   輪胎轉「一整圈」後記錄計數差，即為 PPR (Pulse Per Revolution)。
//
// 做法：
//   1. 上電後 Serial Monitor (115200 baud) 即時顯示四輪編碼器計數
//   2. 用手轉動其中一顆輪胎「剛好一圈」
//   3. 讀取該輪計數值的變化量 = PPR
//   4. 按一下 BOOTSEL button (或送 'r') 可歸零所有計數器
//
// 移植回 move/main.cpp：
//   將測量到的 PPR 常數填入 readEncoders() 函數：
//     constexpr float ENC_COUNTS_PER_REV = 4.0f * <你測到的 PPR>;
//
// 編碼器配置（與 move/main.cpp 一致）：
//   - 腳位：MOTOR1_ENC_A/B = GP10/11, ..., MOTOR4_ENC_A/B = GP16/17
//   - AB 正交編碼器，4 倍頻解碼：每一步 edge 計數 ±1
// ============================================================================
#include <Arduino.h>
#include <hardware/timer.h>
#include <pico/stdlib.h>
#include <cstdio>

// ============================================================================
// 腳位定義（與 move/main.cpp 完全一致）
// ============================================================================
namespace Pin {
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
// 輪子名稱（方便辨識）
// ============================================================================
static const char* WHEEL_NAMES[4] = {"FL", "FR", "RL", "RR"};

// ============================================================================
// GPIO IRQ 軟體編碼器 — 4 倍頻 AB 相解碼
//
// 雖然 move/main.cpp 使用 PIO 硬體編碼器，但 PIO 的 X 暫存器
// 無法從 ARM Core 端直接讀取，無法用於顯示即時計數。
// 此處改用 GPIO 中斷 + 軟體解碼，量測出的 PPR 值與 PIO 編碼器
// （使用相同的 4 倍頻 AB 正交解碼邏輯）完全相同，可直接移植。
// ============================================================================
static volatile int32_t swCount[4] = {0, 0, 0, 0};
static uint8_t prevAB[4] = {0, 0, 0, 0};

// GPIO IRQ handler (所有 encoder pin 共用)
void __not_in_flash_func(encoderGpioIrq)() {
    for(int i = 0; i < 4; i++) {
        uint pinA, pinB;
        switch(i) {
        case 0: pinA = Pin::MOTOR1_ENC_A; pinB = Pin::MOTOR1_ENC_B; break;
        case 1: pinA = Pin::MOTOR2_ENC_A; pinB = Pin::MOTOR2_ENC_B; break;
        case 2: pinA = Pin::MOTOR3_ENC_A; pinB = Pin::MOTOR3_ENC_B; break;
        case 3: pinA = Pin::MOTOR4_ENC_A; pinB = Pin::MOTOR4_ENC_B; break;
        default: return;
        }

        // 檢查是否有 IRQ 觸發
        // cppcheck-suppress badBitmaskCheck — mock 回傳 0，真實硬體回傳位元遮罩
        uint32_t irqStatus = gpio_get_irq_event_mask(pinA) |
                             gpio_get_irq_event_mask(pinB);
        if(!irqStatus) continue;

        // 清除 IRQ flag
        gpio_acknowledge_irq(pinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);
        gpio_acknowledge_irq(pinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL);

        // 讀取 A, B 當前電位
        uint8_t a = gpio_get(pinA) ? 1 : 0;
        uint8_t b = gpio_get(pinB) ? 1 : 0;
        uint8_t ab = (a << 1) | b;

        // Gray code 狀態機 (4 倍頻解碼)
        // 狀態轉換表：每一步 ±1 代表方向
        static const int8_t encTable[4][4] = {
            // 00    01    11    10  (new)
            { 0,   +1,   -1,    0 }, // 前一個 = 00
            { -1,   0,   +1,    0 }, // 前一個 = 01
            { 0,   -1,    0,   +1 }, // 前一個 = 11
            { +1,   0,   -1,    0 }, // 前一個 = 10
        };

        int8_t delta = encTable[prevAB[i]][ab];
        if(delta != 0) {
            swCount[i] += delta;
        }
        prevAB[i] = ab;
    }
}

void initSwEncoder(int idx, uint pinA, uint pinB) {
    gpio_init(pinA);
    gpio_init(pinB);
    gpio_set_dir(pinA, GPIO_IN);
    gpio_set_dir(pinB, GPIO_IN);
    gpio_pull_up(pinA);
    gpio_pull_up(pinB);

    // 雙邊緣觸發中斷
    gpio_set_irq_enabled(pinA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(pinB, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    // 讀取初始狀態
    uint8_t a = gpio_get(pinA) ? 1 : 0;
    uint8_t b = gpio_get(pinB) ? 1 : 0;
    prevAB[idx] = (a << 1) | b;

    Serial.print("[PPR Test] Encoder ");
    Serial.print(idx);
    Serial.print(" (");
    Serial.print(WHEEL_NAMES[idx]);
    Serial.print("): A=GP");
    Serial.print(pinA);
    Serial.print(", B=GP");
    Serial.print(pinB);
    Serial.println(" IRQ ready.");
}

// ============================================================================
// 歸零
// ============================================================================
static void resetAllCounters() {
    for(int i = 0; i < 4; i++) {
        swCount[i] = 0;
    }
    Serial.println("\n>>> Counters RESET <<<");
}

// ============================================================================
// 顯示格式
// ============================================================================
static void printHeader() {
    Serial.println("\n");
    Serial.println("====================================================");
    Serial.println("  PPR 量測工具 — 用『手』轉輪胎一圈");
    Serial.println("====================================================");
    Serial.println("  送 'r' → 歸零");
    Serial.println("  送 'p' → 暫停/繼續");
    Serial.println("  按 BOOTSEL → 歸零");
    Serial.println("----------------------------------------------------");
    Serial.println("        FL (GP10/11)      FR (GP12/13)");
    Serial.println("        RL (GP14/15)      RR (GP16/17)");
    Serial.println("----------------------------------------------------");
    Serial.print("  時間");
    Serial.print("    FL 計數 (+/s)");
    Serial.print("    FR 計數 (+/s)");
    Serial.print("    RL 計數 (+/s)");
    Serial.println("    RR 計數 (+/s)");
    Serial.println("----------------------------------------------------");
}

// ============================================================================
// setup() — 初始化
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n[PPR Test] Booting...");

    // 初始化四輪軟體編碼器
    initSwEncoder(0, Pin::MOTOR1_ENC_A, Pin::MOTOR1_ENC_B);
    initSwEncoder(1, Pin::MOTOR2_ENC_A, Pin::MOTOR2_ENC_B);
    initSwEncoder(2, Pin::MOTOR3_ENC_A, Pin::MOTOR3_ENC_B);
    initSwEncoder(3, Pin::MOTOR4_ENC_A, Pin::MOTOR4_ENC_B);

    // 註冊 IRQ handler
    irq_set_exclusive_handler(IO_IRQ_BANK0, encoderGpioIrq);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // BOOTSEL button: RP2350 上一般為 GP21 (active low)
    gpio_init(21);
    gpio_set_dir(21, GPIO_IN);
    gpio_pull_up(21);

    printHeader();
}

// ============================================================================
// loop() — 主迴圈
// ============================================================================
void loop() {
    static uint64_t lastPrint = 0;
    static bool paused = false;
    static int32_t prevCounts[4] = {0, 0, 0, 0};
    static uint64_t startMs = 0;

    // ---- Serial 命令處理 ----
    while(Serial.available()) {
        char c = Serial.read();
        if(c == 'r' || c == 'R') {
            resetAllCounters();
            for(int i = 0; i < 4; i++) prevCounts[i] = 0;
            startMs = millis();
        }
        if(c == 'p' || c == 'P') {
            paused = !paused;
            Serial.print(paused ? "\n>>> PAUSED <<<\n" : "\n>>> RESUMED <<<\n");
        }
    }

    // ---- BOOTSEL button polling (歸零) ----
    static bool lastBtn = true;
    bool btn = gpio_get(21);
    if(lastBtn && !btn) {
        resetAllCounters();
        for(int i = 0; i < 4; i++) prevCounts[i] = 0;
        startMs = millis();
        delay(200);
    }
    lastBtn = btn;

    if(paused) {
        delay(100);
        return;
    }

    // ---- 每 200ms 更新一次顯示 ----
    uint64_t nowMs = millis();
    if(nowMs - lastPrint < 200) return;
    lastPrint = nowMs;
    if(startMs == 0) startMs = nowMs;

    // 覆寫當前行
    static int lineCount = 0;
    if(lineCount++ > 30) {
        lineCount = 0;
        printHeader();
    }

    // 計算秒速
    float elapsedSec = (nowMs - startMs) / 1000.0f;

    Serial.print("\r");
    for(int i = 0; i < 4; i++) {
        int32_t c = swCount[i];
        int32_t delta = c - prevCounts[i];
        prevCounts[i] = c;

        // 每秒速率 = delta / 0.2s * 1s = delta * 5
        int32_t rate = delta * 5;

        char buf[40];
        snprintf(buf, sizeof(buf), "%s %+8d (+%4d/s) | ",
                 WHEEL_NAMES[i], static_cast<int>(c), static_cast<int>(rate));
        Serial.print(buf);
    }

    // 顯示運作時間
    Serial.printf("[%.0fs]", elapsedSec);
    Serial.flush();
}
