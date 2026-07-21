#include "ChassisComms.h"

ChassisComms::ChassisComms(Stream* stream,
                           uint8_t hbTarget,
                           uint64_t hbIntervalUs)
    : stream(stream),
      hbTargetID(hbTarget),
      hbIntervalUs(hbIntervalUs) {}

void ChassisComms::begin() {
    // 重設 COBS 掃描狀態
    workBufIdx   = 0;
    firstFrame   = true;
    lastFrameSeq = 0;

    // 重設心跳計時器
    lastHeartbeatTime = 0;
    txFrameSeq        = 0;
}

void ChassisComms::processFrame(const uint8_t* data, size_t len) {
    // Check for telemetry frame (larger than ControlPacket)
    if (len == sizeof(MoveTelemetry)) {
        const MoveTelemetry& tm = *reinterpret_cast<const MoveTelemetry*>(data);
        if (tm.sync == 0xA5 && tm.msgType == MsgType::TELEMETRY) {
            if (telemetryCb != nullptr)
                telemetryCb(tm);
            return;
        }
    }

    if (len != sizeof(ControlPacket)) return;

    const ControlPacket& pkt = *reinterpret_cast<const ControlPacket*>(data);
    if (pkt.sync != 0xA5) return;

    // E-Stop 為中斷事件：跳過幀序號去重，每次都必須傳遞（CLAUDE.md §E-Stop）
    if (pkt.msgType == MsgType::ESTOP) {
        if (frameCb != nullptr)
            frameCb(pkt);
        return;
    }

    // 幀序號抗重複 (CLAUDE.md §幀序號抗干擾)
    if (!firstFrame && pkt.frameSeq == lastFrameSeq)
        return;
    firstFrame   = false;
    lastFrameSeq = pkt.frameSeq;

    if (frameCb != nullptr)
        frameCb(pkt);
}

bool ChassisComms::sendTelemetry(const MoveTelemetry& telemetry) {
    if (stream == nullptr) return false;

    uint8_t  cobsBuf[sizeof(MoveTelemetry) + 2];
    size_t   cobsLen = SignalUtils::cobsEncode(
                          reinterpret_cast<const uint8_t*>(&telemetry),
                          sizeof(MoveTelemetry),
                          cobsBuf);
    cobsBuf[cobsLen] = 0x00;

    size_t totalBytes = cobsLen + 1;

    if (stream->availableForWrite() >= static_cast<int>(totalBytes)) {
        size_t written = stream->write(cobsBuf, totalBytes);
        return written == totalBytes;
    }
    return false;
}

bool ChassisComms::sendPacket(const ControlPacket& pkt) {
    if (stream == nullptr) return false;

    uint8_t  cobsBuf[sizeof(ControlPacket) + 2];
    size_t   cobsLen = SignalUtils::cobsEncode(
                          reinterpret_cast<const uint8_t*>(&pkt),
                          sizeof(ControlPacket),
                          cobsBuf);
    cobsBuf[cobsLen] = 0x00;

    size_t totalBytes = cobsLen + 1;

    // 非阻塞寫入：檢查 TX 剩餘空間 (CLAUDE.md §發送規則)
    if (stream->availableForWrite() >= static_cast<int>(totalBytes)) {
        size_t written = stream->write(cobsBuf, totalBytes);
        return written == totalBytes;
    }
    return false; // TX 滿 → 靜默丟棄
}

void ChassisComms::update() {
    // ---- 1. 從綁定的 Stream 讀取 ----
    if (stream == nullptr) return;

    int avail = stream->available();
    if (avail > 0) {
        // 每次最多處理 64 bytes，避免佔用 loop() 太久
        int toRead = (avail > 64) ? 64 : avail;
        for (int i = 0; i < toRead; i++) {
            int byte = stream->read();
            if (byte < 0) break;

            uint8_t b = static_cast<uint8_t>(byte);

            if (b == 0x00) {
                // COBS 幀結束 — 嘗試解碼
                if (workBufIdx >= sizeof(ControlPacket)) {
                    uint8_t decoded[512];
                    size_t  decLen = SignalUtils::cobsDecode(parseWorkBuf,
                                                              workBufIdx,
                                                              decoded);
                    if (decLen >= sizeof(ControlPacket)) {
                        uint16_t storedCrc =
                            reinterpret_cast<const ControlPacket*>(
                                decoded)->crc16;
                        uint16_t calcCrc = SignalUtils::calculateCrc(
                                               decoded,
                                               sizeof(ControlPacket) - 2);
                        if (storedCrc == calcCrc) {
                            processFrame(decoded, decLen);
                        }
                        // CRC 失敗 → 靜默丟棄 (CLAUDE.md 規則)
                    }
                }
                workBufIdx = 0;
            } else {
                if (workBufIdx < sizeof(parseWorkBuf) - 1)
                    parseWorkBuf[workBufIdx++] = b;
            }
        }
    }

    // ---- 2. 心跳 (CLAUDE.md §心跳) ----
    if (hbIntervalUs == 0) return; // 停用心跳

    uint64_t now = time_us_64();
    if (now - lastHeartbeatTime >= hbIntervalUs) {
        lastHeartbeatTime = now;

        ControlPacket pkt;
        pkt.sync       = 0xA5;
        pkt.msgType    = MsgType::HEARTBEAT;
        pkt.frameSeq   = txFrameSeq++;
        pkt.targetID   = hbTargetID;
        pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;
        pkt.move.vx    = 0;
        pkt.move.vy    = 0;
        pkt.move.omega = 0;

        pkt.crc16 = SignalUtils::calculateCrc(
                        reinterpret_cast<uint8_t*>(&pkt),
                        sizeof(ControlPacket) - 2);

        sendPacket(pkt);
    }
}
