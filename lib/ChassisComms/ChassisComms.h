#pragma once
#include "signal_shared.h"
#include <Arduino.h>
#include <hardware/dma.h>
#include <hardware/uart.h>

// ============================================================================
// ChassisComms — Hub ↔ Jetson UART communication (Hub-side only)
//
// Per agent.md topology:
//   Hub ↔ Jetson: hardware UART0, GP0 TX, GP1 RX, 921600 baud
//
// Handles:
//   - DMA ping-pong receive (lock-free, zero-copy scan)
//   - COBS frame decode + CRC-CCITT verify
//   - Frame sequence anti-duplicate (agent.md §帧序号抗干扰)
//   - Registered callback for decoded ControlPacket dispatch
//   - 30ms heartbeat generation (agent.md §心跳)
//   - Non-blocking TX with drop-on-full (agent.md §发送规则)
//
// NOT for slave nodes — Slaves connect via PIO UART.
// ============================================================================
class ChassisComms {
public:
    static ChassisComms& getInstance();

    void init();
    void update();

    /// Callback invoked for every valid decoded ControlPacket from Jetson.
    using FrameCallback = void (*)(const ControlPacket& pkt);
    void setFrameCallback(FrameCallback cb) { frameCb = cb; }

    /// Send a ControlPacket to Jetson. Non-blocking — drops if TX FIFO full.
    /// Returns true if queued, false if dropped.
    bool sendPacket(const ControlPacket& pkt);

private:
    ChassisComms() = default;
    ChassisComms(const ChassisComms&) = delete;
    ChassisComms& operator=(const ChassisComms&) = delete;

    void initUartDmaPingPong();
    void processFrame(const uint8_t* data, size_t len);

    // DMA ping-pong — static to stay in .bss
    static uint8_t dmaBuffer[2][512];
    static int     dmaChanA;
    static int     dmaChanB;

    // COBS scan state
    size_t  scanPos    = 0;
    uint8_t parseWorkBuf[512];
    size_t  workBufIdx = 0;

    // Frame receive tracking
    FrameCallback frameCb     = nullptr;
    uint8_t       lastFrameSeq = 0;
    bool          firstFrame   = true;

    // Heartbeat state
    uint64_t lastHeartbeatTime = 0;
    uint8_t  txFrameSeq       = 0;
};
