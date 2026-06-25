#include "ChassisComms.h"

// ============================================================================
// Static member definitions (.bss allocation)
// ============================================================================
uint8_t ChassisComms::dmaBuffer[2][512];
int     ChassisComms::dmaChanA;
int     ChassisComms::dmaChanB;

ChassisComms& ChassisComms::getInstance() {
    static ChassisComms instance;
    return instance;
}

// ============================================================================
// Hardware config — per agent.md UART table:
//   Hub ↔ Jetson: UART0, GP0 TX (Pin 1), GP1 RX (Pin 2), 921600 baud
// ============================================================================
static uart_inst_t* const   HUB_UART   = uart0;
static constexpr uint8_t    HUB_TX_PIN = 0;   // GP0 = Pin 1
static constexpr uint8_t    HUB_RX_PIN = 1;   // GP1 = Pin 2
static const uint32_t       HUB_BAUD   = RobotConfig::JETSON_BAUDRATE;

// ============================================================================
// UART + DMA ping-pong init
// Two DMA channels chain to each other: when buffer A fills (512 bytes),
// DMA auto-starts buffer B. We scan the inactive buffer in update().
// ============================================================================
void ChassisComms::initUartDmaPingPong() {
    // UART0 GPIO function — use SDK native API
    uart_init(HUB_UART, HUB_BAUD);
    gpio_set_function(HUB_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HUB_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enabled(HUB_UART, true);

    // Claim DMA channels
    dmaChanA = dma_claim_unused_channel(true);
    dmaChanB = dma_claim_unused_channel(true);

    // Channel A → dmaBuffer[0]
    {
        dma_channel_config cA = dma_channel_get_default_config(dmaChanA);
        channel_config_set_transfer_data_size(&cA, DMA_SIZE_8);
        channel_config_set_read_increment(&cA,  false);
        channel_config_set_write_increment(&cA, true);
        channel_config_set_dreq(&cA, uart_get_dreq(HUB_UART, false));
        channel_config_set_chain_to(&cA, dmaChanB);

        dma_channel_configure(
            dmaChanA, &cA,
            dmaBuffer[0],                     // write address
            &uart_get_hw(HUB_UART)->dr,        // read address (RX FIFO)
            512,                                // count
            false                               // don't start yet
        );
    }

    // Channel B → dmaBuffer[1]
    {
        dma_channel_config cB = dma_channel_get_default_config(dmaChanB);
        channel_config_set_transfer_data_size(&cB, DMA_SIZE_8);
        channel_config_set_read_increment(&cB,  false);
        channel_config_set_write_increment(&cB, true);
        channel_config_set_dreq(&cB, uart_get_dreq(HUB_UART, false));
        channel_config_set_chain_to(&cB, dmaChanA);

        dma_channel_configure(
            dmaChanB, &cB,
            dmaBuffer[1],
            &uart_get_hw(HUB_UART)->dr,
            512,
            false
        );
    }

    // Start: DMA writes into buffer A first
    dma_channel_start(dmaChanA);
}

// ============================================================================
// Process a fully decoded (COBS-decoded + CRC-verified) frame
//   - frameSeq anti-duplicate check (agent.md §帧序号抗干扰)
//   - Dispatch to registered callback
// ============================================================================
void ChassisComms::processFrame(const uint8_t* data, size_t len) {
    if (len != sizeof(ControlPacket)) return;

    // Reinterpret as ControlPacket
    const ControlPacket& pkt = *reinterpret_cast<const ControlPacket*>(data);

    // Validate sync byte
    if (pkt.sync != 0xA5) return;

    // FrameSeq duplicate check — skip if same as last valid frame
    if (!firstFrame && pkt.frameSeq == lastFrameSeq)
        return;  // Duplicate or retransmission
    firstFrame    = false;
    lastFrameSeq  = pkt.frameSeq;

    // Dispatch to registered handler (if any)
    if (frameCb != nullptr)
        frameCb(pkt);
}

// ============================================================================
// ChassisComms::init()
// Called once from setup() to initialize CRC table, UART, and DMA.
// ============================================================================
void ChassisComms::init() {
    initUartDmaPingPong();
}

// ============================================================================
// ChassisComms::sendPacket()
// COBS-encode + CRC-stamp a ControlPacket and write to UART.
// Per agent.md §发送规则: drops on full TX, no blocking.
// ============================================================================
bool ChassisComms::sendPacket(const ControlPacket& pkt) {
    uint8_t  cobsBuf[sizeof(ControlPacket) + 2]; // worst-case COBS overhead
    size_t   cobsLen = SignalUtils::cobsEncode(
                          reinterpret_cast<const uint8_t*>(&pkt),
                          sizeof(ControlPacket),
                          cobsBuf);

    // Append delimiter
    cobsBuf[cobsLen] = 0x00;

    // Non-blocking write — drop if TX full
    if (uart_is_writable(HUB_UART)) {
        // Write all bytes via raw UART (no Serial abstraction — less overhead)
        for (size_t i = 0; i <= cobsLen; i++) {
            // uart_putc_raw — no FIFO wait, the TX FIFO has room because
            // we checked uart_is_writable above. At 921600 baud the FIFO
            // drains faster than we can fill 20 bytes.
            uart_putc_raw(HUB_UART, cobsBuf[i]);
        }
        return true;
    }
    return false; // TX full — dropped
}

// ============================================================================
// ChassisComms::update()
// Per-invocation slice:
//   1. Determine which DMA buffer is inactive (safe to read)
//   2. Scan <= 64 new bytes for COBS delimiters (0x00)
//   3. On delimiter: COBS decode → CRC verify → processFrame()
//   4. Every 30ms: generate heartbeat to Jetson
//
// Total budget per call: < 200μs at 921600 baud (agent.md rule).
// ============================================================================
void ChassisComms::update() {
    // ---- 1. Determine inactive DMA buffer ----
    uint32_t currentActive = dma_channel_is_busy(dmaChanA) ? 1 : 0;
    uint8_t* readBuffer    = dmaBuffer[currentActive ^ 1];

    // ---- 2. Scan for COBS frames (max 64 bytes / call) ----
    size_t processed = 0;
    while (scanPos < 512 && processed < 64) {
        uint8_t byte = readBuffer[scanPos++];
        processed++;

        if (byte == 0x00) {
            // End of COBS frame — attempt decode if we have meaningful data
            if (workBufIdx >= sizeof(ControlPacket)) {
                // COBS decode into a temporary buffer
                uint8_t decoded[512];
                size_t  decLen = SignalUtils::cobsDecode(parseWorkBuf,
                                                          workBufIdx,
                                                          decoded);
                if (decLen >= sizeof(ControlPacket)) {
                    // CRC verify (all fields EXCEPT the crc16 bytes)
                    uint16_t storedCrc = reinterpret_cast<const ControlPacket*>(
                                             decoded)->crc16;
                    uint16_t calcCrc   = SignalUtils::calculateCrc(
                                             decoded,
                                             sizeof(ControlPacket) - 2);
                    if (storedCrc == calcCrc) {
                        processFrame(decoded, decLen);
                    }
                    // CRC failure → silently drop (agent.md rule)
                }
            }
            workBufIdx = 0; // Reset for next frame
        } else {
            if (workBufIdx < sizeof(parseWorkBuf) - 1)
                parseWorkBuf[workBufIdx++] = byte;
        }
    }

    // Wrap scan position
    if (scanPos >= 512)
        scanPos = 0;

    // ---- 3. Heartbeat — 30ms interval (agent.md §心跳) ----
    uint64_t now = time_us_64();
    if (now - lastHeartbeatTime >= RobotConfig::HEARTBEAT_INTERVAL_US) {
        lastHeartbeatTime = now;

        ControlPacket pkt;
        pkt.sync       = 0xA5;
        pkt.msgType    = MsgType::HEARTBEAT;
        pkt.frameSeq   = txFrameSeq++;
        pkt.targetID   = TargetID::HUB;
        pkt.payloadLen = sizeof(ControlPacket) - 6;  // remaining after header
        pkt.vx         = 0;
        pkt.vy         = 0;
        pkt.omega      = 0;
        pkt.mode       = RobotMode::FAILSAFE;
        pkt.timestamp_hi = static_cast<uint32_t>(now >> 32);
        pkt.timestamp_lo = static_cast<uint32_t>(now & 0xFFFFFFFF);
        pkt.crc16      = 0; // zero then calculate

        pkt.crc16 = SignalUtils::calculateCrc(
                        reinterpret_cast<uint8_t*>(&pkt),
                        sizeof(ControlPacket) - 2);

        sendPacket(pkt);
    }
}
