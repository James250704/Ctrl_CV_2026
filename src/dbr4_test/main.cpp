#include <Arduino.h>
#include <CrsfReceiver.h> // 直接 include 解耦後的通用庫

// 宣告遙控接收機物件
CrsfReceiver crsf;

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("[HUB] 機器人中央主控啟動...");

    // 初始化遙控器：使用 UART0，接收腳位 GP1 (Pico 上的 Pin 2)
    crsf.begin(0, 1);
}

unsigned long lastLogTime = 0;

void loop() {
    // 1. 讓接收機大腦保持運轉（內部自動處理解包、濾波、Failsafe 與 LED）
    crsf.update();

    // 2. 獲取最後常態化處理好的 [-100 ~ 100] 控制數據
    int16_t rc_data[10];
    crsf.getChannels(rc_data);

    // 3. 每 100 毫秒印出一次當前控制權狀態，用於與 Jetson 混控除錯
    unsigned long now = millis();
    if(now - lastLogTime >= 50) {
        lastLogTime = now;

        if(crsf.isFailsafe()) {
            Serial.println(">>> [狀態: 遙控器斷線保護中] 馬達已強制安全歸零！");
        } else {
            Serial.printf(">>> [狀態: 遙控手動控制] %d, %d, %d, %d, %d, %d, "
                          "%d, %d, %d, %d\n",
                          rc_data[0], rc_data[1], rc_data[2], rc_data[3],
                          rc_data[4], rc_data[5], rc_data[6], rc_data[7],
                          rc_data[8], rc_data[9]);

            // TODO: 在這裡直接使用 rc_data 數值去控制你的底盤馬達！
            // 例如：motor_move(rc_data[2], rc_data[0]);
        }
    }

    delay(1); // 給背景通訊留點喘息時間
}