#include "ChassisComms.h" // 引入封裝好的通訊庫
#include <Arduino.h>

// 取得通訊模組單例實例
ChassisComms& chassisComms = ChassisComms::getInstance();

// ============================================================================
// Core 0: 負責高層邏輯調度、通訊協定解析
// ============================================================================
void setup() {
    // 初始化通訊模組（含控制硬體腳位與啟動 DMA 雙快取）
    chassisComms.init();

    // 初始化板載 LED 作為狀態指示
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    // 輪詢處理通訊狀態機與發送 30ms 心跳包
    chassisComms.update();

    // 保持極短的延遲，避免阻塞 Core 0
    delayMicroseconds(20);
}

// ============================================================================
// Core 1: 保留給對即時性要求極高的底盤電機 PID 運算 (麥克納姆輪控制等)
// ============================================================================
void setup1() {
    // 這裡可以初始化馬達相關驅動，如：
    // MotorControl::init();
}

void loop1() {
    // 這裡可以放入即時性較高的馬達控制環，與 Core 0 獨立分流
    // MotorControl::updatePID();
}