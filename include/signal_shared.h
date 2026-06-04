#pragma once
#include <Arduino.h>

// ============================================================================
// A. 系統硬體與通訊參數常數配置
// ============================================================================
namespace RobotConfig {
constexpr uint32_t JETSON_BAUDRATE = 115200;
constexpr uint32_t CRSF_BAUDRATE = 420000;
constexpr uint32_t CHASSIS_BAUDRATE = 921600;

constexpr uint32_t CONTROL_DT_MS = 1;
constexpr uint64_t COMMAND_TIMEOUT_US = 100000;
constexpr uint64_t HEARTBEAT_TIMEOUT_US = 500000;

constexpr float MAX_RPS = 7.0f;
constexpr float KF_GAIN = 21.5f;
constexpr float PID_OUT_LIMIT = 30.0f;
constexpr float SLEW_RATE_LIMIT = 50.0f;
constexpr uint16_t MOTOR_MAX_DUTY = 40000;
} // namespace RobotConfig

namespace RobotMode {
constexpr uint8_t MANUAL = 0;
constexpr uint8_t AUTO = 1;
constexpr uint8_t FAILSAFE = 2;
} // namespace RobotMode

// ============================================================================
// C. 二進位通訊封包資料結構 (完全平鋪設計，徹底消滅巢狀 Padding)
// ============================================================================
#pragma pack(push, 1)

struct ControlPacket {
    // Header (4 bytes)
    uint8_t sync;        // 1 byte  (固定為 0xA5)
    uint8_t msgType;     // 1 byte  (固定為 0x01)
    uint16_t payloadLen; // 2 bytes (固定為 12)

    // Payload (平鋪展開 - 12 bytes)
    int8_t vx;             // 1 byte
    int8_t vy;             // 1 byte
    int8_t omega;          // 1 byte
    uint8_t mode;          // 1 byte
    uint32_t timestamp_hi; // 4 bytes
    uint32_t timestamp_lo; // 4 bytes

    // Tail (2 bytes)
    uint16_t crc16; // 2 bytes
};

#pragma pack(pop)

// 最終防線檢查：1 + 1 + 2 + 1 + 1 + 1 + 1 + 4 + 4 + 2 = 18 單位位元組！
// 注意：平鋪展開後，全體總長度本身就是 18
// Bytes。我們將靜態斷言修正為真實物理大小。
static_assert(sizeof(ControlPacket) == 18,
              "ControlPacket struct packing size mismatched!");

struct ChassisSnapshot {
    float vx;
    float vy;
    float omega;
    uint8_t mode;
    uint64_t lastArrivalTick;
};

// ============================================================================
// D. 工具函數
// ============================================================================
namespace SignalUtils {
inline void initCrcTable(uint16_t* table) {
    for(int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for(int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
        table[i] = crc;
    }
}

inline uint16_t calculateCrc(const uint16_t* table, const uint8_t* data,
                             size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ table[((crc >> 8) ^ data[i]) & 0xFF];
    }
    return crc;
}

inline size_t cobsEncode(const uint8_t* src, size_t len, uint8_t* dst) {
    size_t code_idx = 0;
    size_t read_idx = 0;
    size_t write_idx = 1;
    uint8_t code = 1;

    while(read_idx < len) {
        if(src[read_idx] == 0x00) {
            dst[code_idx] = code;
            code = 1;
            code_idx = write_idx++;
            read_idx++;
        } else {
            dst[write_idx++] = src[read_idx++];
            code++;
            if(code == 0xFF) {
                dst[code_idx] = code;
                code = 1;
                code_idx = write_idx++;
            }
        }
    }
    dst[code_idx] = code;
    return write_idx;
}

inline size_t cobsDecode(const uint8_t* src, size_t len, uint8_t* dst) {
    size_t read_idx = 0;
    size_t write_idx = 0;
    while(read_idx < len) {
        uint8_t code = src[read_idx];
        if(read_idx + code > len && code != 1)
            return 0;
        read_idx++;
        for(uint8_t i = 1; i < code; i++) {
            dst[write_idx++] = src[read_idx++];
        }
        if(code < 0xFF && read_idx < len) {
            dst[write_idx++] = 0x00;
        }
    }
    return write_idx;
}
} // namespace SignalUtils