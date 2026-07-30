#pragma once
// MockStream — 可控制的 Stream 實作，用於測試 ChassisComms
//
// 使用方式：
//   MockStream stream;
//   ChassisComms comms(&stream, TargetID::HUB);
//   // 注入 bytes 供 comms 讀取：
//   stream.rxPush(data, len);
//   comms.update();
//   // 驗證 comms 發送的 bytes：
//   stream.txRead(captured, capturedLen);

#include "Arduino.h"
#include <vector>
#include <cstdint>

class MockStream : public Stream {
public:
    // ---- RX (host → device) ----
    void rxPush(const uint8_t* data, size_t len) {
        rxBuf.insert(rxBuf.end(), data, data + len);
    }
    void rxPushByte(uint8_t b) {
        rxBuf.push_back(b);
    }
    int available() override {
        return static_cast<int>(rxBuf.size() - rxReadIdx);
    }
    int read() override {
        if (rxReadIdx >= rxBuf.size()) return -1;
        return rxBuf[rxReadIdx++];
    }
    void rxReset() {
        rxBuf.clear();
        rxReadIdx = 0;
    }

    // ---- TX (device → host) ----
    size_t write(uint8_t c) override {
        txBuf.push_back(c);
        return 1;
    }
    size_t write(const uint8_t* data, size_t len) override {
        txBuf.insert(txBuf.end(), data, data + len);
        return len;
    }
    int availableForWrite() override {
        return 4096 - static_cast<int>(txBuf.size());
    }
    void flush() override {}

    // ---- TX 讀取（驗證用） ----
    size_t txAvailable() const {
        return txBuf.size() - txReadIdx;
    }
    size_t txRead(uint8_t* dst, size_t maxLen) {
        size_t avail = txBuf.size() - txReadIdx;
        size_t toRead = (avail < maxLen) ? avail : maxLen;
        memcpy(dst, txBuf.data() + txReadIdx, toRead);
        txReadIdx += toRead;
        return toRead;
    }
    uint8_t txReadByte() {
        if (txReadIdx >= txBuf.size()) return 0;
        return txBuf[txReadIdx++];
    }
    void txReset() {
        txBuf.clear();
        txReadIdx = 0;
    }

private:
    std::vector<uint8_t> rxBuf;
    size_t rxReadIdx = 0;
    std::vector<uint8_t> txBuf;
    size_t txReadIdx = 0;
};
