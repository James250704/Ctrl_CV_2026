// ============================================================================
// test_chassis_comms.cpp — ChassisComms send/receive/heartbeat 單元測試
//
// 使用 MockStream 模擬 Serial/SerialPIO，驗證 COBS 封包收發流程。
// ============================================================================

#include <unity.h>
#include "MockStream.h"
#include "ChassisComms.h"

// ---- Helper: extract COBS frame from MockStream TX buffer ----
// COBS frame: encoded bytes + 0x00 terminator
static size_t extractCobsFrame(MockStream& stream, uint8_t* dst, size_t dstMax) {
    size_t txAvail = stream.txAvailable();
    if (txAvail == 0) return 0;

    uint8_t raw[1024];
    size_t rawLen = stream.txRead(raw, txAvail);

    // Find 0x00 terminator
    for (size_t i = 0; i < rawLen; i++) {
        if (raw[i] == 0x00) {
            // Copy encoded bytes (without terminator)
            size_t frameLen = i;
            if (frameLen > dstMax) frameLen = dstMax;
            memcpy(dst, raw, frameLen);
            // Discard the 0x00 and any remaining bytes (skip past terminator)
            stream.txReset();
            // Re-push remaining bytes after the 0x00
            if (i + 1 < rawLen) {
                stream.txReset();
                // We've already read everything, just reset
            }
            return frameLen;
        }
    }
    // No terminator found — incomplete frame
    return 0;
}

// ---- Test: sendPacket produces valid COBS with CRC ----
void test_sendPacket_valid_cobs(void) {
    MockStream stream;
    ChassisComms comms(&stream, TargetID::HUB, 0);  // no heartbeat
    comms.begin();

    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = 1;
    pkt.targetID   = TargetID::MOVE;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;
    pkt.move.vx    = 10;
    pkt.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - 2);

    bool sent = comms.sendPacket(pkt);
    TEST_ASSERT_TRUE(sent);

    // TX buffer should contain COBS encoded bytes + 0x00 terminator
    size_t txLen = stream.txAvailable();
    TEST_ASSERT_TRUE(txLen > sizeof(ControlPacket));  // COBS adds overhead

    // Read all TX bytes
    uint8_t raw[256];
    size_t rawLen = stream.txRead(raw, sizeof(raw));
    TEST_ASSERT_TRUE(rawLen > 0);
    TEST_ASSERT_EQUAL_HEX8(0x00, raw[rawLen - 1]);  // terminator

    // All bytes except last should be non-zero (COBS property)
    for (size_t i = 0; i < rawLen - 1; i++) {
        TEST_ASSERT_NOT_EQUAL(0x00, raw[i]);
    }
}

// ---- Test: receive control frame triggers callback ----
static bool frameCallbackCalled = false;
static ControlPacket lastReceivedFrame;

static void onTestFrame(const ControlPacket& pkt) {
    frameCallbackCalled = true;
    lastReceivedFrame = pkt;
}

void test_receive_control_frame(void) {
    MockStream stream;
    ChassisComms comms(&stream, TargetID::HUB, 0);
    comms.begin();
    comms.setFrameCallback(onTestFrame);
    frameCallbackCalled = false;

    // Build a valid ControlPacket and COBS encode it
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = 1;
    pkt.targetID   = TargetID::HUB;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;
    pkt.move.vx    = 42;
    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - 2);

    // COBS encode + add terminator
    uint8_t encoded[256];
    size_t encLen = SignalUtils::cobsEncode(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket), encoded);
    encoded[encLen] = 0x00;  // terminator

    // Inject into mock stream
    stream.rxPush(encoded, encLen + 1);

    // Process
    comms.update();

    TEST_ASSERT_TRUE(frameCallbackCalled);
    TEST_ASSERT_EQUAL_HEX8(0xA5, lastReceivedFrame.sync);
    TEST_ASSERT_EQUAL_INT8(42, lastReceivedFrame.move.vx);
}

// ---- Test: receive telemetry frame triggers telemetry callback ----
static bool telemetryCallbackCalled = false;
static MoveTelemetry lastReceivedTm;

static void onTestTelemetry(const MoveTelemetry& tm) {
    telemetryCallbackCalled = true;
    lastReceivedTm = tm;
}

void test_receive_telemetry_frame(void) {
    MockStream stream;
    ChassisComms comms(&stream, TargetID::HUB, 0);
    comms.begin();
    comms.setTelemetryCallback(onTestTelemetry);
    telemetryCallbackCalled = false;

    // Build a valid MoveTelemetry
    MoveTelemetry tm = {};
    tm.sync       = 0xA5;
    tm.msgType    = MsgType::TELEMETRY;
    tm.frameSeq   = 5;
    tm.targetID   = TargetID::HUB;
    tm.currentRps[0] = 3.14f;
    tm.timestamp  = 99999ULL;
    tm.crc16 = 0;
    tm.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&tm),
        sizeof(MoveTelemetry) - 2);

    // COBS encode
    uint8_t encoded[256];
    size_t encLen = SignalUtils::cobsEncode(
        reinterpret_cast<const uint8_t*>(&tm),
        sizeof(MoveTelemetry), encoded);
    encoded[encLen] = 0x00;  // terminator

    stream.rxPush(encoded, encLen + 1);

    // update() reads max 64 bytes per call; 74-byte frame needs 2+ calls
    for (int i = 0; i < 10 && !telemetryCallbackCalled; i++) {
        comms.update();
    }

    TEST_ASSERT_TRUE(telemetryCallbackCalled);
    TEST_ASSERT_EQUAL_FLOAT(3.14f, lastReceivedTm.currentRps[0]);
    TEST_ASSERT_EQUAL_HEX64(99999ULL, lastReceivedTm.timestamp);
}

// ---- Test: heartbeat generation ----
void test_heartbeat_generation(void) {
    MockStream stream;
    // Use tiny interval (1us) so heartbeat fires almost immediately
    ChassisComms comms(&stream, TargetID::HUB, 1);
    comms.begin();

    // Run update until heartbeat appears or we give up
    for (int i = 0; i < 100; i++) {
        comms.update();
        if (stream.txAvailable() > 0) break;
    }

    size_t txLen = stream.txAvailable();
    if (txLen == 0) {
        printf("\n    [DEBUG] NO heartbeat generated\n");
    }
    TEST_ASSERT_TRUE(txLen > 0);

    // Read all TX bytes
    uint8_t raw[256];
    size_t rawLen = stream.txRead(raw, sizeof(raw));

    // Find terminator
    size_t frameEnd = 0;
    for (size_t i = 0; i < rawLen; i++) {
        if (raw[i] == 0x00) { frameEnd = i; break; }
    }
    TEST_ASSERT_TRUE(frameEnd > 0);

    // COBS decode
    uint8_t decoded[256];
    size_t decLen = SignalUtils::cobsDecode(raw, frameEnd, decoded);
    TEST_ASSERT_EQUAL_size_t(sizeof(ControlPacket), decLen);

    const ControlPacket& hb = *reinterpret_cast<const ControlPacket*>(decoded);
    TEST_ASSERT_EQUAL_HEX8(0xA5, hb.sync);
    TEST_ASSERT_EQUAL_HEX8(MsgType::HEARTBEAT, hb.msgType);
    TEST_ASSERT_EQUAL_HEX8(TargetID::HUB, hb.targetID);
}

// ---- Test: frame dedup ----
void test_frame_dedup(void) {
    MockStream stream;
    ChassisComms comms(&stream, TargetID::HUB, 0);
    comms.begin();
    comms.setFrameCallback(onTestFrame);
    frameCallbackCalled = false;

    // Build a control frame
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = 7;  // same seq both times
    pkt.targetID   = TargetID::HUB;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;
    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket) - 2);

    uint8_t encoded[256];
    size_t encLen = SignalUtils::cobsEncode(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket), encoded);
    encoded[encLen] = 0x00;

    // Inject same frame twice
    stream.rxPush(encoded, encLen + 1);
    comms.update();
    TEST_ASSERT_TRUE(frameCallbackCalled);

    frameCallbackCalled = false;
    stream.rxPush(encoded, encLen + 1);
    comms.update();
    // Second same frameSeq should be deduped
    TEST_ASSERT_FALSE(frameCallbackCalled);
}

// ---- Test: CRC mismatch is silently dropped ----
void test_crc_mismatch_dropped(void) {
    MockStream stream;
    ChassisComms comms(&stream, TargetID::HUB, 0);
    comms.begin();
    comms.setFrameCallback(onTestFrame);
    frameCallbackCalled = false;

    // Build a control frame with WRONG CRC
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = 1;
    pkt.targetID   = TargetID::HUB;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;
    pkt.crc16      = 0xDEAD;  // wrong CRC!

    uint8_t encoded[256];
    size_t encLen = SignalUtils::cobsEncode(
        reinterpret_cast<const uint8_t*>(&pkt),
        sizeof(ControlPacket), encoded);
    encoded[encLen] = 0x00;

    stream.rxPush(encoded, encLen + 1);
    comms.update();

    // Should NOT trigger callback
    TEST_ASSERT_FALSE(frameCallbackCalled);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_sendPacket_valid_cobs);
    RUN_TEST(test_receive_control_frame);
    RUN_TEST(test_receive_telemetry_frame);
    RUN_TEST(test_heartbeat_generation);
    RUN_TEST(test_frame_dedup);
    RUN_TEST(test_crc_mismatch_dropped);

    return UNITY_END();
}
