#ifndef CHASSIS_COMMS_H
#define CHASSIS_COMMS_H

#include "signal_shared.h"
#include <Arduino.h>
#include <atomic>
#include <hardware/dma.h>
#include <hardware/uart.h>

class ChassisComms {
  public:
    static ChassisComms& getInstance() {
        static ChassisComms instance;
        return instance;
    }

    void init();
    void update();

  private:
    ChassisComms() = default;

    void initUartDmaPingPong();

    // DMA 靜態快取分配
    static uint8_t dmaBuffer[2][512];
    static int dmaChanA, dmaChanB;

    // 狀態機成員變數（取代原本 loop 內的 static 變數）
    size_t scanPos = 0;
    uint8_t parseWorkBuf[512];
    size_t workBufIdx = 0;
    uint64_t lastHeartbeatTime = 0;
};

#endif // CHASSIS_COMMS_H