#include "signal_shared.h"
#include <Arduino.h>
#include <atomic>
#include <hardware/dma.h>
#include <hardware/uart.h>

// 校準項目 7: 明確指定實體硬體引腳與配置
#define CHASSIS_UART_HARDWARE uart0
#define CHASSIS_UART_SERIAL Serial1
#define CHASSIS_TX_PIN 16 // GP16 = Pin 21
#define CHASSIS_RX_PIN 17 // GP17 = Pin 22

// DMA Ping-Pong 靜態快取分配 (各 512 位元組)
static uint8_t dmaBuffer[2][512];
static int dmaChanA, dmaChanB;
static std::atomic<uint32_t> activeDmaBuf(0);

void initUartDmaPingPong() {
    // 配置 UART 數據傳輸至 DMA
    uart_set_fifo_enabled(CHASSIS_UART_HARDWARE, true);

    dmaChanA = dma_claim_unused_channel(true);
    dmaChanB = dma_claim_unused_channel(true);

    // 通道 A 配置 (寫入 dmaBuffer[0])
    dma_channel_config cA = dma_channel_get_default_config(dmaChanA);
    channel_config_set_transfer_data_size(&cA, DMA_SIZE_8);
    channel_config_set_read_increment(&cA, false); // 固定讀取 UART RX FIFO
    channel_config_set_write_increment(&cA, true); // 自動累加寫入 Buffer
    channel_config_set_dreq(&cA, uart_get_dreq(CHASSIS_UART_HARDWARE, false));
    channel_config_set_chain_to(&cA, dmaChanB); // 滿了自動啟動 B 通道

    dma_channel_configure(dmaChanA, &cA, dmaBuffer[0],
                          &uart_get_hw(CHASSIS_UART_HARDWARE)->dr, 512, false);

    // 通道 B 配置 (寫入 dmaBuffer[1])
    dma_channel_config cB = dma_channel_get_default_config(dmaChanB);
    channel_config_set_transfer_data_size(&cB, DMA_SIZE_8);
    channel_config_set_read_increment(&cB, false);
    channel_config_set_write_increment(&cB, true);
    channel_config_set_dreq(&cB, uart_get_dreq(CHASSIS_UART_HARDWARE, false));
    channel_config_set_chain_to(&cB, dmaChanA); // 滿了自動換回 A 通道

    dma_channel_configure(dmaChanB, &cB, dmaBuffer[1],
                          &uart_get_hw(CHASSIS_UART_HARDWARE)->dr, 512, false);

    // 啟動首發 DMA
    dma_channel_start(dmaChanA);
}

void setup() {
    // 校準項目 15: 初始化全局靜態 CrcTable
    SignalUtils::initCrcTable();

    // 校準項目 7: 嚴格依照 Spec 指定實體硬體腳位初始化 UART
    CHASSIS_UART_SERIAL.setTX(CHASSIS_TX_PIN);
    CHASSIS_UART_SERIAL.setRX(CHASSIS_RX_PIN);
    CHASSIS_UART_SERIAL.begin(RobotConfig::CHASSIS_BAUDRATE);

    initUartDmaPingPong();
    pinMode(LED_BUILTIN, OUTPUT);
}

// Core 0 分批解析狀態機
void loop() {
    static size_t scanPos = 0;
    static uint8_t parseWorkBuf[512];
    static size_t workBufIdx = 0;

    // 獲取當前未被 DMA 佔用的唯讀緩衝區索引
    uint32_t currentActive = dma_channel_is_busy(dmaChanA) ? 1 : 0;
    uint8_t* readBuffer = dmaBuffer[currentActive ^ 1];

    // 每次限制處理最多 64 bytes，保證單次 loop 週期 < 200μs
    size_t processedBytes = 0;
    while(scanPos < 512 && processedBytes < 64) {
        uint8_t byte = readBuffer[scanPos++];
        processedBytes++;

        if(byte == 0x00) { // 尋獲分包終止符
            if(workBufIdx > sizeof(ControlPacket)) {
                // 解碼與執行處理 (細節由通訊混合狀態機接尾)
            }
            workBufIdx = 0; // 重置
        } else {
            if(workBufIdx < 511)
                parseWorkBuf[workBufIdx++] = byte;
        }
    }

    if(scanPos >= 512)
        scanPos = 0;

    // ============================================================================
    // 校準項目 6: 落實通訊心跳定時器閘控 (每 30ms 強制發送心跳幀至下位機)
    // ============================================================================
    static uint64_t lastHeartbeatTime = 0;
    uint64_t now = time_us_64();

    if(now - lastHeartbeatTime >= 30000) { // 30ms 週期
        lastHeartbeatTime = now;

        ControlPacket heartbeatPacket;
        heartbeatPacket.sync = 0xA5;
        heartbeatPacket.msgType = 0x01; // 心跳/控制指令同一封包結構
        heartbeatPacket.payloadLen = 12;
        heartbeatPacket.vx = 0;
        heartbeatPacket.vy = 0;
        heartbeatPacket.omega = 0;
        heartbeatPacket.mode = RobotMode::FAILSAFE; // 心跳預設維持安全狀態
        heartbeatPacket.timestamp_hi = (uint32_t)(now >> 32);
        heartbeatPacket.timestamp_lo = (uint32_t)(now & 0xFFFFFFFF);

        heartbeatPacket.crc16 = SignalUtils::calculateCrc(
            reinterpret_cast<uint8_t*>(&heartbeatPacket),
            sizeof(ControlPacket) - 2);

        uint8_t encodeWorkspace[sizeof(ControlPacket) + 2];
        size_t encodedLen = SignalUtils::cobsEncode(
            reinterpret_cast<uint8_t*>(&heartbeatPacket), sizeof(ControlPacket),
            encodeWorkspace);

        // 校準項目 7: 寫入完畢後，必須補齊 0x00 分隔符作為影格結束標誌
        encodeWorkspace[encodedLen] = 0x00;

        if(CHASSIS_UART_SERIAL.availableForWrite() >= (int)(encodedLen + 1)) {
            CHASSIS_UART_SERIAL.write(encodeWorkspace, encodedLen + 1);
        }
    }

    delayMicroseconds(20);
}

void setup1() {}
void loop1() {}