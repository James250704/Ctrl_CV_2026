#pragma once
#include <Arduino.h>

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
// 校準項目 14: 強制使用 __attribute__((packed)) 規範字面要求
// ============================================================================
struct ControlPacket {
    uint8_t sync;
    uint8_t msgType;
    uint16_t payloadLen;
    int8_t vx;
    int8_t vy;
    int8_t omega;
    uint8_t mode;
    uint32_t timestamp_hi;
    uint32_t timestamp_lo;
    uint16_t crc16;
} __attribute__((packed));

static_assert(sizeof(ControlPacket) == 18, "ControlPacket padding violation!");

struct ChassisSnapshot {
    float vx;
    float vy;
    float omega;
    uint8_t mode;
    uint64_t lastArrivalTick;
};

namespace SignalUtils {
// 校準項目 15: 內部變數與全局命名嚴格命名為 crcTable
static uint16_t crcTable[256];

inline void initCrcTable() {
    for(int i = 0; i < 256; i++) {
        uint16_t crc = i << 8;
        for(int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
        crcTable[i] = crc;
    }
}

inline uint16_t calculateCrc(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < len; i++) {
        crc = (crc << 8) ^ crcTable[((crc >> 8) ^ data[i]) & 0xFF];
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