// ============================================================================
// servo_test — 伺服馬達旋轉控制測試程式 (RP2350)
//
// 用途：
//   透過 Serial 命令控制伺服馬達角度，驗證 PWM（GP17）接線與 Servo 函式庫
//   運作是否正常。可逐一微調脈衝寬度範圍以符合實際馬達規格。
//
// 硬體：
//   - 伺服馬達訊號線 → GP17 (Pin 22)
//   - 伺服馬達 VCC → VBUS (5V) 或 VSYS
//   - 伺服馬達 GND → GND
//
// Serial 命令：
//   a<angle>    → 設定角度，例：a90 → 90°，a-45 → -45°
//   w<pulse>    → 直接設定脈衝寬度 (µs)，例：w1500 → 1500µs
//   s           → 掃描：從 0° 到 180° 再回 0°，每次步進 10°/500ms
//   c           → 連續掃描模式下切換啟用/暫停
//   min <us>    → 設定最小脈衝寬度（預設 500µs）
//   max <us>    → 設定最大脈衝寬度（預設 2500µs）
//   home        → 回到 90° (中立點)
//   detach      → 停止 PWM 輸出
//   attach      → 重新啟用 PWM 輸出
//   ? / help    → 顯示命令列表
//
// 接線確認（SG90 / MG996R / 通用）：
//   棕色/黑色 → GND
//   紅色      → 5V
//   橘色/黃色 → 訊號 (GP17)
// ============================================================================
#include <Arduino.h>
#include <Servo.h>

// ============================================================================
// 腳位定義
// ============================================================================
namespace Pin {
constexpr uint SERVO_PWM = 17; // 伺服馬達 PWM 控制腳 (GP17 = Pin 22)
} // namespace Pin

// ============================================================================
// 伺服設定
// ============================================================================
static constexpr int SERVO_FREQ_HZ = 50; // 標準 50Hz

// 預設脈衝範圍（適用多數小型伺服，SG90 約 500~2400µs，MG996R 約 500~2500µs）
static int minPulseUs = 500;
static int maxPulseUs = 2500;
static int currentAngle = 90;

static Servo servo;
static bool attached = false;

// ============================================================================
// 命令處理輔助
// ============================================================================

// 跳過前綴空白並回傳第一個非空白字元指標
static const char* skipSpace(const char* p) {
    while(p && *p == ' ') p++;
    return p;
}

// 解析整數數值，p 會更新到數值結尾
static int parseInt(const char*& p) {
    p = skipSpace(p);
    bool neg = false;
    if(*p == '-') { neg = true; p++; }
    int val = 0;
    while(*p >= '0' && *p <= '9') {
        val = val * 10 + (*p - '0');
        p++;
    }
    return neg ? -val : val;
}

// ============================================================================
// Servo 控制函式
// ============================================================================

static void servoAttach() {
    if(!attached) {
        // 使用自訂 min/max 脈衝寬度
        int ch = servo.attach(Pin::SERVO_PWM, minPulseUs, maxPulseUs);
        if(ch == 0) {
            Serial.println("[ERROR] 無法 attach Servo — 可能無可用 PIO SM");
            return;
        }
        attached = true;
        // 寫入當前角度
        servo.write(currentAngle);
        Serial.printf("[Servo] Attached GP%d, min=%dµs, max=%dµs, angle=%d°\n",
                      Pin::SERVO_PWM, minPulseUs, maxPulseUs, currentAngle);
    } else {
        Serial.println("[Servo] 已經 attached");
    }
}

static void servoDetach() {
    if(attached) {
        servo.detach();
        attached = false;
        Serial.println("[Servo] Detached — PWM 已停止");
    } else {
        Serial.println("[Servo] 已經 detached");
    }
}

static void servoSetAngle(int angle) {
    // 一般伺服支援 -90° ~ 180°（部分 270°）
    // 超出範圍時提示但不阻止
    if(angle < -90 || angle > 270) {
        Serial.printf("[WARN] 角度 %d° 超出常見範圍 (-90~270)\n", angle);
    }
    currentAngle = angle;
    if(attached) {
        servo.write(angle);
        int us = servo.readMicroseconds();
        Serial.printf("[Servo] → %d°  (%dµs)\n", angle, us);
    } else {
        Serial.printf("[Servo] 已記錄角度 %d° (未 attach，呼叫 attach 後生效)\n", angle);
    }
}

static void servoSetPulseUs(int us) {
    if(us < 500 || us > 3000) {
        Serial.printf("[WARN] 脈衝 %dµs 超出常見範圍 (500~3000)\n", us);
    }
    if(attached) {
        servo.writeMicroseconds(us);
        int angle = servo.read();
        Serial.printf("[Servo] → %dµs  (%d°)\n", us, angle);
    } else {
        Serial.printf("[Servo] 無法寫入 — 尚未 attach\n");
    }
}

static void servoSetRange(int minUs, int maxUs) {
    if(minUs >= maxUs || minUs < 300 || maxUs > 3500) {
        Serial.printf("[ERROR] 無效範圍: %d~%d µs\n", minUs, maxUs);
        return;
    }
    minPulseUs = minUs;
    maxPulseUs = maxUs;
    Serial.printf("[Servo] 更新脈衝範圍: min=%dµs, max=%dµs\n", minPulseUs, maxPulseUs);
    if(attached) {
        // 需要重新 attach 才能生效
        servo.detach();
        int ch = servo.attach(Pin::SERVO_PWM, minPulseUs, maxPulseUs);
        if(ch == 0) {
            Serial.println("[ERROR] 重新 attach 失敗");
            attached = false;
            return;
        }
        attached = true;
        servo.write(currentAngle);
        Serial.printf("[Servo] 已重新 attach 新範圍，角度=%d°\n", currentAngle);
    }
}

// ============================================================================
// 掃描模式
// ============================================================================
static bool sweeping = false;

static void doSweep() {
    Serial.println("[Sweep] 開始掃描 0° → 180° → 0°");
    sweeping = true;
    // 先確保已 attach
    if(!attached) servoAttach();
    if(!attached) { sweeping = false; return; }

    // 上升段 0° → 180°
    for(int a = 0; a <= 180; a += 10) {
        servo.write(a);
        Serial.printf("[Sweep] %3d°  (%dµs)\n", a, servo.readMicroseconds());
        delay(500);
        // 檢查是否被命令中斷
        if(!sweeping) {
            Serial.println("[Sweep] 被中斷");
            return;
        }
    }
    // 下降段 180° → 0°
    for(int a = 170; a >= 0; a -= 10) {
        servo.write(a);
        Serial.printf("[Sweep] %3d°  (%dµs)\n", a, servo.readMicroseconds());
        delay(500);
        if(!sweeping) {
            Serial.println("[Sweep] 被中斷");
            return;
        }
    }
    Serial.println("[Sweep] 完成");
    sweeping = false;
}

// ============================================================================
// 說明
// ============================================================================
static void printHelp() {
    Serial.println("\n===== Servo Test 命令列表 =====");
    Serial.println("  a<angle>      → 設定角度，例: a90");
    Serial.println("  w<pulse_us>   → 直接寫入脈衝寬度 (µs)，例: w1500");
    Serial.println("  s             → 單次掃描 0° → 180° → 0° (步進 10°)");
    Serial.println("  c             → 切換連續掃描啟用/暫停");
    Serial.println("  min <us>      → 設定最小脈衝寬度 (預設 500)");
    Serial.println("  max <us>      → 設定最大脈衝寬度 (預設 2500)");
    Serial.println("  home          → 回到中立點 (90°)");
    Serial.println("  attach        → 啟用 PWM 輸出");
    Serial.println("  detach        → 停止 PWM 輸出");
    Serial.println("  ? / help      → 顯示此說明");
    Serial.println("  status        → 顯示當前狀態");
    Serial.println("===============================\n");
}

static void printStatus() {
    Serial.println("\n===== Servo Status =====");
    Serial.printf("  Pin:        GP%d (%s)\n", Pin::SERVO_PWM, attached ? "Attached" : "Detached");
    Serial.printf("  Pulse:      %d ~ %d µs\n", minPulseUs, maxPulseUs);
    if(attached) {
        Serial.printf("  Current:    %d°  (%dµs)\n", servo.read(), servo.readMicroseconds());
    } else {
        Serial.printf("  Recorded:   %d°\n", currentAngle);
    }
    Serial.printf("  Sweeping:   %s\n", sweeping ? "Yes" : "No");
    Serial.println("==========================\n");
}

// ============================================================================
// setup()
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1500);

    Serial.println("\n========================================");
    Serial.println("  Servo Test — 伺服馬達控制測試");
    Serial.println("========================================");
    Serial.printf("  PWM Pin:  GP%d (Pin 22)\n", Pin::SERVO_PWM);
    Serial.printf("  脈衝範圍: %d ~ %d µs\n", minPulseUs, maxPulseUs);
    Serial.println("  輸入 '?' 或 'help' 查看命令列表");
    Serial.println("========================================\n");

    // 自動 attach
    servoAttach();

    // 歸零到 90°
    if(attached) {
        servo.write(90);
        Serial.printf("  Initial: 90° (%dµs)\n", servo.readMicroseconds());
    }
}

// ============================================================================
// loop() — 主迴圈
// ============================================================================
void loop() {
    // ---- Serial 命令處理 ----
    while(Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if(line.length() == 0) continue;

        const char* p = line.c_str();

        // 略過開頭空白後看第一個字
        p = skipSpace(p);
        if(!*p) continue;

        char cmd = *p;

        // 單字元命令
        if(cmd == 'a' || cmd == 'A') {
            p++; // 跳過 'a'
            int angle = parseInt(p);
            servoSetAngle(angle);

        } else if(cmd == 'w' || cmd == 'W') {
            p++; // 跳過 'w'
            int us = parseInt(p);
            servoSetPulseUs(us);

        } else if(cmd == 's' || cmd == 'S') {
            // 單次掃描
            if(!sweeping) {
                doSweep();
            } else {
                Serial.println("[Sweep] 掃描進行中，略過");
            }

        } else if(cmd == 'c' || cmd == 'C') {
            // 切換連續掃描
            sweeping = !sweeping;
            Serial.printf("[Sweep] 連續掃描: %s\n", sweeping ? "啟動" : "暫停");
            if(sweeping && attached) {
                // 在 loop 中執行掃描動畫
            }

        } else if(cmd == '?') {
            printHelp();

        } else {
            // 多字元命令比對
            if(strcmp(p, "help") == 0) {
                printHelp();
            } else if(strcmp(p, "home") == 0) {
                servoSetAngle(90);
            } else if(strcmp(p, "attach") == 0) {
                servoAttach();
            } else if(strcmp(p, "detach") == 0) {
                servoDetach();
            } else if(strcmp(p, "status") == 0) {
                printStatus();
            } else if(strncmp(p, "min ", 4) == 0) {
                const char* val = p + 4;
                int us = parseInt(val);
                servoSetRange(us, maxPulseUs);
            } else if(strncmp(p, "max ", 4) == 0) {
                const char* val = p + 4;
                int us = parseInt(val);
                servoSetRange(minPulseUs, us);
            } else {
                Serial.printf("[ERROR] 未知命令: '%s' (輸入 '?' 查看說明)\n", p);
            }
        }
    }

    // ---- 連續掃描處理 ----
    if(sweeping && attached) {
        static int sweepDir = 1;
        static int sweepAngle = 0;
        static uint32_t lastStep = 0;

        uint32_t now = millis();
        if(now - lastStep >= 30) { // 每 30ms 步進 1° → 約 3 秒完成一次掃描
            lastStep = now;
            sweepAngle += sweepDir;
            if(sweepAngle >= 180) {
                sweepAngle = 180;
                sweepDir = -1;
            } else if(sweepAngle <= 0) {
                sweepAngle = 0;
                sweepDir = 1;
            }
            servo.write(sweepAngle);
        }
    }

    // 避免佔用 CPU 過高
    delay(1);
}
