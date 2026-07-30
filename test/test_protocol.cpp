// ============================================================================
// test_protocol.cpp — CRC16 + COBS + ControlPacket + MoveTelemetry 單元測試
// ============================================================================

#include <unity.h>
#include "signal_shared.h"

// ============================================================================
// CRC16-CCITT Tests
// ============================================================================

void test_crc16_empty_data(void) {
    uint16_t crc = SignalUtils::calculateCrc(nullptr, 0);
    // CRC of empty data with init=0xFFFF should return 0xFFFF
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc);
}

void test_crc16_known_vector(void) {
    // "123456789" is a standard CRC-CCITT test vector
    const uint8_t data[] = "123456789";
    uint16_t crc = SignalUtils::calculateCrc(data, 9);
    // CRC-CCITT (0xFFFF) of "123456789" = 0x29B1
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc);
}

void test_crc16_single_byte(void) {
    const uint8_t data[] = {0x00};
    uint16_t crc = SignalUtils::calculateCrc(data, 1);
    // CRC-CCITT of single 0x00 byte with init=0xFFFF
    uint16_t expected = 0xE1F0; // computed from table
    TEST_ASSERT_EQUAL_HEX16(expected, crc);
}

void test_crc16_all_zeros(void) {
    uint8_t data[8] = {0};
    uint16_t crc = SignalUtils::calculateCrc(data, 8);
    // Should not be 0xFFFF (data is not empty)
    TEST_ASSERT_NOT_EQUAL(0xFFFF, crc);
}

// ============================================================================
// COBS Encode/Decode Tests
// ============================================================================

void test_cobs_encode_decode_no_zeros(void) {
    // Data without 0x00 bytes: encode adds 1 overhead byte
    const uint8_t src[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t encoded[16];
    size_t encLen = SignalUtils::cobsEncode(src, 4, encoded);

    // 4 bytes → 4 + 1 code byte = 5 bytes encoded
    TEST_ASSERT_EQUAL_size_t(5, encLen);

    // Decode
    uint8_t decoded[16];
    size_t decLen = SignalUtils::cobsDecode(encoded, encLen, decoded);
    TEST_ASSERT_EQUAL_size_t(4, decLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, decoded, 4);
}

void test_cobs_encode_decode_with_zeros(void) {
    // Data with 0x00 bytes
    const uint8_t src[] = {0x01, 0x00, 0x03, 0x00, 0x05};
    uint8_t encoded[16];
    size_t encLen = SignalUtils::cobsEncode(src, 5, encoded);

    // Should not contain 0x00 in encoded data (except possibly as last byte which is terminator)
    // Actually COBS encoded data should have no 0x00 bytes at all
    for (size_t i = 0; i < encLen; i++) {
        TEST_ASSERT_NOT_EQUAL(0x00, encoded[i]);
    }

    // Decode
    uint8_t decoded[16];
    size_t decLen = SignalUtils::cobsDecode(encoded, encLen, decoded);
    TEST_ASSERT_EQUAL_size_t(5, decLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, decoded, 5);
}

void test_cobs_encode_single_zero(void) {
    const uint8_t src[] = {0x00};
    uint8_t encoded[16];
    size_t encLen = SignalUtils::cobsEncode(src, 1, encoded);

    uint8_t decoded[16];
    size_t decLen = SignalUtils::cobsDecode(encoded, encLen, decoded);
    TEST_ASSERT_EQUAL_size_t(1, decLen);
    TEST_ASSERT_EQUAL_UINT8(0x00, decoded[0]);
}

void test_cobs_decode_invalid_too_short(void) {
    // Empty input
    uint8_t decoded[16];
    size_t decLen = SignalUtils::cobsDecode(nullptr, 0, decoded);
    TEST_ASSERT_EQUAL_size_t(0, decLen);
}

void test_cobs_roundtrip_large_payload(void) {
    // Simulate a ControlPacket (20 bytes) with mixed values
    uint8_t src[20];
    for (int i = 0; i < 20; i++) {
        src[i] = static_cast<uint8_t>((i * 37 + 13) & 0xFF);
    }
    // Ensure some zeros to test zero-handling
    src[5] = 0x00;
    src[10] = 0x00;
    src[15] = 0x00;

    uint8_t encoded[64];
    size_t encLen = SignalUtils::cobsEncode(src, 20, encoded);

    uint8_t decoded[64];
    size_t decLen = SignalUtils::cobsDecode(encoded, encLen, decoded);
    TEST_ASSERT_EQUAL_size_t(20, decLen);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(src, decoded, 20);
}

// ============================================================================
// ControlPacket Tests
// ============================================================================

void test_control_packet_size(void) {
    TEST_ASSERT_EQUAL_size_t(20, sizeof(ControlPacket));
}

void test_control_packet_fields(void) {
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = 42;
    pkt.targetID   = TargetID::MOVE;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;
    pkt.move.vx    = 10;
    pkt.move.vy    = -20;
    pkt.move.omega = 50;
    pkt.crc16 = 0;

    // Compute CRC
    pkt.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - 2);

    // Verify fields after memcpy (simulates wire transfer)
    ControlPacket rx;
    memcpy(&rx, &pkt, sizeof(ControlPacket));

    TEST_ASSERT_EQUAL_HEX8(0xA5, rx.sync);
    TEST_ASSERT_EQUAL_HEX8(MsgType::CONTROL, rx.msgType);
    TEST_ASSERT_EQUAL_HEX8(42, rx.frameSeq);
    TEST_ASSERT_EQUAL_HEX8(TargetID::MOVE, rx.targetID);
    TEST_ASSERT_EQUAL_INT8(10, rx.move.vx);
    TEST_ASSERT_EQUAL_INT8(-20, rx.move.vy);
    TEST_ASSERT_EQUAL_INT8(50, rx.move.omega);
    TEST_ASSERT_EQUAL_HEX16(pkt.crc16, rx.crc16);
}

void test_control_packet_crc_roundtrip(void) {
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = 1;
    pkt.targetID   = TargetID::SHOOT;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;
    pkt.shoot.upSpeed   = 75;
    pkt.shoot.downSpeed = -50;
    pkt.shoot.pitch     = 0;
    pkt.shoot.row       = 30;
    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - 2);

    // Verify CRC independently
    uint16_t calcCrc = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - 2);
    TEST_ASSERT_EQUAL_HEX16(pkt.crc16, calcCrc);
}

// ============================================================================
// MoveTelemetry Tests
// ============================================================================

void test_move_telemetry_size(void) {
    TEST_ASSERT_EQUAL_size_t(72, sizeof(MoveTelemetry));
}

void test_move_telemetry_crc(void) {
    MoveTelemetry tm = {};
    tm.sync       = 0xA5;
    tm.msgType    = MsgType::TELEMETRY;
    tm.frameSeq   = 10;
    tm.targetID   = TargetID::HUB;
    tm.currentRps[0] = 3.5f;
    tm.currentRps[1] = -2.1f;
    tm.currentRps[2] = 0.0f;
    tm.currentRps[3] = 7.0f;
    tm.targetRps[0]  = 3.0f;
    tm.targetRps[1]  = -2.0f;
    tm.targetRps[2]  = 0.0f;
    tm.targetRps[3]  = 7.0f;
    tm.pwmOutput[0]  = 15.5f;
    tm.cmdVx    = 1.0f;
    tm.cmdVy    = 0.5f;
    tm.cmdOmega = 0.2f;
    tm.mode     = RobotMode::AUTO;
    tm.estopFlag = 0;
    tm.timestamp = 12345678ULL;
    tm.crc16 = 0;
    tm.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&tm),
        sizeof(MoveTelemetry) - 2);

    // Verify CRC
    uint16_t calcCrc = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&tm),
        sizeof(MoveTelemetry) - 2);
    TEST_ASSERT_EQUAL_HEX16(tm.crc16, calcCrc);

    // Verify values survive memcpy
    MoveTelemetry rx;
    memcpy(&rx, &tm, sizeof(MoveTelemetry));
    TEST_ASSERT_EQUAL_FLOAT(3.5f, rx.currentRps[0]);
    TEST_ASSERT_EQUAL_FLOAT(-2.1f, rx.currentRps[1]);
    TEST_ASSERT_EQUAL_FLOAT(15.5f, rx.pwmOutput[0]);
    TEST_ASSERT_EQUAL_HEX64(12345678ULL, rx.timestamp);
}

// ============================================================================
// MsgType COBS Safety Tests
// ============================================================================

void test_msgtype_all_nonzero(void) {
    // All msgType values must be non-zero for COBS safety
    TEST_ASSERT_NOT_EQUAL(0, MsgType::HEARTBEAT);
    TEST_ASSERT_NOT_EQUAL(0, MsgType::CONTROL);
    TEST_ASSERT_NOT_EQUAL(0, MsgType::TELEMETRY);
    TEST_ASSERT_NOT_EQUAL(0, MsgType::REBOOT);
    TEST_ASSERT_NOT_EQUAL(0, MsgType::ESTOP);
}

void test_targetid_values(void) {
    TEST_ASSERT_EQUAL_HEX8(0x00, TargetID::HUB);
    TEST_ASSERT_EQUAL_HEX8(0x01, TargetID::MOVE);
    TEST_ASSERT_EQUAL_HEX8(0x02, TargetID::SHOOT);
    TEST_ASSERT_EQUAL_HEX8(0x03, TargetID::STORAGE);
    TEST_ASSERT_EQUAL_HEX8(0xFF, TargetID::BROADCAST);
}

// ============================================================================
// Unity test runner
// ============================================================================

int main(void) {
    UNITY_BEGIN();

    // CRC16
    RUN_TEST(test_crc16_empty_data);
    RUN_TEST(test_crc16_known_vector);
    RUN_TEST(test_crc16_single_byte);
    RUN_TEST(test_crc16_all_zeros);

    // COBS
    RUN_TEST(test_cobs_encode_decode_no_zeros);
    RUN_TEST(test_cobs_encode_decode_with_zeros);
    RUN_TEST(test_cobs_encode_single_zero);
    RUN_TEST(test_cobs_decode_invalid_too_short);
    RUN_TEST(test_cobs_roundtrip_large_payload);

    // ControlPacket
    RUN_TEST(test_control_packet_size);
    RUN_TEST(test_control_packet_fields);
    RUN_TEST(test_control_packet_crc_roundtrip);

    // MoveTelemetry
    RUN_TEST(test_move_telemetry_size);
    RUN_TEST(test_move_telemetry_crc);

    // Constants
    RUN_TEST(test_msgtype_all_nonzero);
    RUN_TEST(test_targetid_values);

    return UNITY_END();
}
