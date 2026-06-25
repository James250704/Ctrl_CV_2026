#pragma once
#include <cstdint>
#include <cstddef>
#include <Arduino.h>

// ============================================================================
// RobotConfig — System-level constants (baud rates, timing, motor limits)
// Per agent.md topology:
//   Hub ↔ Jetson:  hardware UART0 (GP0/GP1)
//   Hub ↔ Slaves:  PIO UART (GP16/GP17 etc.)
//   Debug:         hardware UART1 (GP4/GP5)
// ============================================================================
namespace RobotConfig {

// Baud rates matching agent.md UART table
constexpr uint32_t JETSON_BAUDRATE  = 921600U;  // Hub ↔ Jetson: hardware UART0
constexpr uint32_t PIO_BAUDRATE     = 921600U;  // Hub ↔ Slaves: PIO UART
constexpr uint32_t DEBUG_BAUDRATE   = 115200U;   // Debug UART1 (GP4 TX, GP5 RX)

// Timing — agent.md §超时保护 & §心跳
constexpr uint32_t  CONTROL_DT_MS          = 1U;
constexpr uint64_t  COMMAND_TIMEOUT_US     = 100000ULL;     // 100ms → E-Stop
constexpr uint64_t  HEARTBEAT_TIMEOUT_US   = 500000ULL;     // 500ms
constexpr uint64_t  HEARTBEAT_INTERVAL_US  = 30000ULL;      // 30ms TX interval

// Motor / PID limits
constexpr float     MAX_RPS         = 7.0f;
constexpr float     KF_GAIN         = 21.5f;
constexpr float     PID_OUT_LIMIT   = 30.0f;
constexpr float     SLEW_RATE_LIMIT = 50.0f;
constexpr uint16_t  MOTOR_MAX_DUTY  = 40000U;

// COBS protocol limits
constexpr size_t    COBS_MAX_PAYLOAD     = 254U;
constexpr size_t    COBS_ENCODED_MAX     = 256U;

} // namespace RobotConfig

// ============================================================================
// Protocol constants — per agent.md §Binary Struct
// ============================================================================

namespace MsgType {
constexpr uint8_t HEARTBEAT  = 0x01;
constexpr uint8_t CONTROL    = 0x02;
constexpr uint8_t TELEMETRY  = 0x03;
constexpr uint8_t ESTOP      = 0xFF;
} // namespace MsgType

namespace TargetID {
constexpr uint8_t HUB       = 0x00;
constexpr uint8_t MOVE      = 0x01;
constexpr uint8_t SHOOT     = 0x02;
constexpr uint8_t STORAGE   = 0x03;
constexpr uint8_t BROADCAST = 0xFF;
} // namespace TargetID

namespace RobotMode {
constexpr uint8_t MANUAL   = 0;
constexpr uint8_t AUTO     = 1;
constexpr uint8_t FAILSAFE = 2;
} // namespace RobotMode

// ============================================================================
// ControlPacket — wire-format packet per agent.md protocol spec
//
// Header: sync(0xA5), msgType, frameSeq, targetID, payloadLen, crc16
// CRC16 covers all fields EXCEPT the crc16 field itself.
// Little-endian byte order (RP2350 native).
// ============================================================================
struct ControlPacket {
    uint8_t  sync;         // 0xA5 frame sync byte
    uint8_t  msgType;      // MsgType constant
    uint8_t  frameSeq;     // Incrementing sequence (wraps at 0) — anti-duplicate
    uint8_t  targetID;     // TargetID constant (0=Hub, 1=Move, 2=Shoot, 3=Storage, 0xFF=Broadcast)
    uint16_t payloadLen;   // Payload byte count (fixed = 12 for this struct layout)
    int8_t   vx;           // Linear velocity X
    int8_t   vy;           // Linear velocity Y
    int8_t   omega;        // Angular velocity
    uint8_t  mode;         // RobotMode
    uint32_t timestamp_hi; // Microsecond timestamp (high 32 bits)
    uint32_t timestamp_lo; // Microsecond timestamp (low 32 bits)
    uint16_t crc16;        // CRC-CCITT over all preceding bytes
} __attribute__((packed));

static_assert(sizeof(ControlPacket) == 20, "ControlPacket padding violation!");

// ============================================================================
// ChassisSnapshot — in-memory decoded command (not wire format)
// Used internally for core-to-core command passing per agent.md §核心間通訊
// ============================================================================
struct ChassisSnapshot {
    float    vx;
    float    vy;
    float    omega;
    uint8_t  mode;
    uint8_t  frameSeq;
    uint64_t lastArrivalTick;
};

// ============================================================================
// SignalUtils — CRC-CCITT (poly 0x1021) and COBS encode/decode
//
// CRC table is a compile-time const in Flash — only one copy ever exists
// across all TUs. No initCrcTable() call needed, zero .bss waste.
// ============================================================================
namespace SignalUtils {

// CRC-CCITT lookup table (polynomial 0x1021), ROMable in Flash.
// constexpr guarantees compile-time evaluation — zero runtime init,
// enables aggressive inlining and prefetch by the compiler.
static constexpr uint16_t crcTable[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};

// Calculate CRC-CCITT (0xFFFF init) over data.
// Note: pass len = sizeof(ControlPacket) - 2 to exclude the crc16 field.
inline uint16_t calculateCrc(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc = static_cast<uint16_t>(
            (crc << 8) ^ crcTable[((crc >> 8) ^ data[i]) & 0xFF]
        );
    }
    return crc;
}

// COBS encode: returns encoded byte count.
// dst must be at least len + len / 254 + 2 bytes.
inline size_t cobsEncode(const uint8_t* src, size_t len, uint8_t* dst) {
    size_t codeIdx  = 0;
    size_t readIdx  = 0;
    size_t writeIdx = 1;
    uint8_t code    = 1;

    while (readIdx < len) {
        if (src[readIdx] == 0x00) {
            dst[codeIdx] = code;
            code         = 1;
            codeIdx      = writeIdx++;
            readIdx++;
        } else {
            dst[writeIdx++] = src[readIdx++];
            code++;
            if (code == 0xFF) {
                dst[codeIdx] = code;
                code         = 1;
                codeIdx      = writeIdx++;
            }
        }
    }
    dst[codeIdx] = code;
    return writeIdx;
}

// COBS decode: returns decoded byte count (0 on error).
//
// ** Defence-in-depth **
//   1. Before entering the inner copy loop, verify that we have enough
//      source bytes remaining: readIdx + (code-1) <= len
//   2. Track writeIdx against a 512-byte ceiling (all callers use 512-byte
//      dst buffers). This catches corrupted `code` values from EMI bit-flips
//      that would otherwise overflow dst and trigger a HardFault.
//
// Both guards return 0 (decode failure) — the caller drops the frame.
static constexpr size_t COBS_DST_MAX = 512;

inline size_t cobsDecode(const uint8_t* src, size_t len, uint8_t* dst) {
    size_t readIdx  = 0;
    size_t writeIdx = 0;

    while (readIdx < len) {
        uint8_t code = src[readIdx];
        size_t  blockLen = static_cast<size_t>(code);

        // Guard 1: not enough source bytes for the advertised block
        if (readIdx + blockLen > len) {
            return 0;
        }
        readIdx++;

        // Guard 2: destination would overflow (EMI-corrupted code)
        // blockLen == 1  → no data bytes to copy (just a zero-run marker)
        // blockLen >= 2  → we will write (blockLen-1) data bytes
        if (writeIdx + (blockLen - 1) > COBS_DST_MAX) {
            return 0;
        }

        for (uint8_t i = 1; i < code; i++)
            dst[writeIdx++] = src[readIdx++];

        // Insert the implicit zero (unless this was the final block marker)
        if (code < 0xFF && readIdx < len) {
            if (writeIdx >= COBS_DST_MAX)
                return 0;
            dst[writeIdx++] = 0x00;
        }
    }
    return writeIdx;
}

} // namespace SignalUtils
