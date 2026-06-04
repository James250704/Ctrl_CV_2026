#include "signal_shared.h"

static uint16_t localCrcTable[256];
ControlPacket packet;

void setup() {
    Serial.begin(921600);
    SignalUtils::initCrcTable(localCrcTable);

    // 直接給平鋪後的欄位賦值，乾淨俐落
    packet.sync = 0xA5;
    packet.msgType = 0x01;
    packet.payloadLen = 12; // 展開後的 Payload 實體大小
    packet.vx = 50;
    packet.vy = 0;
    packet.omega = 0;
    packet.mode = RobotMode::AUTO;

    uint64_t now = time_us_64();
    packet.timestamp_hi = (uint32_t)(now >> 32);
    packet.timestamp_lo = (uint32_t)(now & 0xFFFFFFFF);

    packet.crc16 = SignalUtils::calculateCrc(
        localCrcTable, reinterpret_cast<uint8_t*>(&packet),
        sizeof(ControlPacket) - 2);
}

void loop() {
    // 狀態機運作...
}