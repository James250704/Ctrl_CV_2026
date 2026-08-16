// ============================================================================
// rc_move_test — 最小化 RC → Move 通訊測試 (RP2040)
//
// 目的：逐步驗證「遙控器 → PIO UART → Move Slave」這條通道。
//      不接 Jetson、不接其他 Slave，只測兩件事：
//         1. 確認 DBR4 (CRSF 接收機) 能解出遙控通道值 (-100~100)
//         2. 把 CH1/CH2/CH4 構建成 ControlPacket(target=MOVE)，
//            用 PIO 模擬 UART 送到 Move Slave (GP2 TX)
//
// 接線（依 CLAUDE.md 規格）：
//   DBR4 TX     → Hub GP1 (UART0 RX)         420000 baud (CRSF)
//   Hub GP2     → Move RX(GP1)  [PIO SM TX]   921600 baud (COBS)
//   Hub GP3     → Move TX(GP0)  [PIO SM RX]   ← 接收 Move 遙測
//
// 在本機 Serial (USB CDC, 115200) 印出除錯資訊。
//
// 指令對照（CLAUDE.md §RC 通道分配）：
//   CH1 (Roll)  → vx      左右平移
//   CH2 (Pitch) → vy      前後移動
//   CH4 (Yaw)   → omega   旋轉
//   CH6 (AUX2)  → 安全開關（-100 → 強制歸零 / E-Stop）
//
// 覆用現有庫：
//   - CrsfReceiver  : DBR4 解包（UART0 ISR，單核 loop 即可）
//   - ChassisComms  : COBS + 封包格式 + CRC，綁定 SerialPIO
// ============================================================================

#include "ChassisComms.h"
#include <Arduino.h>
#include <CrsfReceiver.h>
#include <SerialPIO.h>

// ============================================================================
// 腳位分配（RP2040）
// ============================================================================
static constexpr uint8_t PIN_CRSF_RX  = 1;  // UART0 RX → DBR4 TX, 420000 baud
static constexpr uint8_t PIN_MOVE_TX  = 2;  // PIO TX → Move slave RX(GP1)
static constexpr uint8_t PIN_MOVE_RX  = 3;  // PIO RX ← Move slave TX(GP0)

// ============================================================================
// 硬體物件
// ============================================================================
static CrsfReceiver crsf;
static SerialPIO moveUart(PIN_MOVE_TX, PIN_MOVE_RX);   // TX=GP2, RX=GP3
static ChassisComms moveComms(&moveUart, TargetID::MOVE, 0); // 無心跳

// ============================================================================
// PIO UART RX 緩衝區 — 接收 Move 回傳的 COBS 遙測幀 (0x00 切割)
// ============================================================================
static constexpr uint16_t PIO_RX_BUF_SIZE = 512;
struct PioRxBuffer { uint8_t buf[PIO_RX_BUF_SIZE]; uint16_t len; };
static PioRxBuffer moveRxBuf;

// 遙測資料儲存
static MoveTelemetry latestMoveTm = {};
static uint64_t lastMoveTmUs = 0;   // 0 = 從未收到
static uint64_t lastPioRxLogUs = 0; // RX 除錯 log 節流

// ============================================================================
// 死區 + 限幅：-10~10 → 0，並映射到 int8
// ============================================================================
static int8_t rcToInt8(int16_t ch) {
    if (ch > -10 && ch < 10) ch = 0;
    if (ch > 100) ch = 100;
    if (ch < -100) ch = -100;
    return static_cast<int8_t>(ch);
}

// ============================================================================
// 建立 Move ControlPacket from RC 通道值
// ============================================================================
static ControlPacket buildMovePacket() {
    int16_t ch[10];
    crsf.getChannels(ch);

    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = static_cast<uint8_t>(millis() / 10); // 控制週期序號
    pkt.targetID   = TargetID::MOVE;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    // CH6 (安全開關) = -100 → 強制歸零 (E-Stop)
    if (ch[5] < 0) {
        pkt.move.vx = pkt.move.vy = pkt.move.omega = 0;
    } else {
        pkt.move.vx    = rcToInt8(ch[0]);   // CH1 → vx
        pkt.move.vy    = rcToInt8(ch[1]);   // CH2 → vy
        pkt.move.omega = rcToInt8(ch[3]);   // CH4 → omega
    }

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
                    reinterpret_cast<const uint8_t*>(&pkt),
                    sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// 處理 PIO UART RX：COBS 解碼 + CRC 驗證 → 存入 latestMoveTm
// ============================================================================
static void processMoveRx() {
    while (moveUart.available() > 0) {
        uint8_t byte = static_cast<uint8_t>(moveUart.read());

        if (byte == 0x00) {
            if (moveRxBuf.len == 0) continue; // 空幀

            uint8_t decoded[512];
            size_t decodedLen = SignalUtils::cobsDecode(moveRxBuf.buf,
                                                        moveRxBuf.len, decoded);
            moveRxBuf.len = 0;
            if (decodedLen != sizeof(MoveTelemetry)) continue;

            uint16_t storedCrc = static_cast<uint16_t>(decoded[decodedLen-2])
                               | (static_cast<uint16_t>(decoded[decodedLen-1]) << 8);
            uint16_t calcCrc = SignalUtils::calculateCrc(decoded, decodedLen-2);
            if (calcCrc != storedCrc) {
                Serial.printf("[RX] CRC fail: calc=0x%04X stored=0x%04X\n",
                              calcCrc, storedCrc);
                continue;
            }

            memcpy(&latestMoveTm, decoded, sizeof(MoveTelemetry));
            lastMoveTmUs = time_us_64();
        } else {
            if (moveRxBuf.len < PIO_RX_BUF_SIZE) {
                moveRxBuf.buf[moveRxBuf.len++] = byte;
            } else {
                moveRxBuf.len = 0; // 溢位 → 丟棄
            }
        }
    }
}

void setup() {
    Serial.begin(115200);   // 本機除錯 (USB CDC)
    delay(500);
    Serial.println("[RC→Move test] 啟動");

    // DBR4 CRSF 接收機 — UART0, GP1 RX, 420000 baud
    crsf.begin(0, PIN_CRSF_RX, 420000);

    // PIO UART 到 Move — 921600 baud, COBS
    moveUart.begin(RobotConfig::PIO_BAUDRATE);
    moveComms.begin();

    moveRxBuf.len = 0;
}

void loop() {
    // 1. 不停餵給 DBR4 解包器
    crsf.update();

    // 2. 從遙控數據構建 Move 控制幀 → PIO UART 送出 (約 100Hz)
    ControlPacket pkt = buildMovePacket();
    moveComms.sendPacket(pkt);

    // 3. 接收 Move 回傳的遙測
    processMoveRx();

    // 4. 每 100ms 印一次狀態
    static unsigned long lastLog = 0;
    unsigned long now = millis();
    if (now - lastLog >= 100) {
        lastLog = now;

        if (crsf.isFailsafe()) {
            Serial.println("[RC] FAILSAFE — 遙控器斷線");
        } else {
            int16_t ch[10];
            crsf.getChannels(ch);
            Serial.printf("[RC] CH0=%3d CH1=%3d CH2=%3d CH3=%3d CH4=%3d "
                          "CH5=%3d CH6=%3d CH7=%3d CH8=%3d CH9=%3d\n",
                          ch[0], ch[1], ch[2], ch[3], ch[4],
                          ch[5], ch[6], ch[7], ch[8], ch[9]);
        }

        if (lastMoveTmUs > 0) {
            Serial.printf(
                "[TM] rps=[%+.2f %+.2f %+.2f %+.2f] ts=%lu\n",
                latestMoveTm.currentRps[0], latestMoveTm.currentRps[1],
                latestMoveTm.currentRps[2], latestMoveTm.currentRps[3],
                static_cast<unsigned long>(latestMoveTm.timestamp));
        } else {
            Serial.println("[TM] 尚未收到 Move 遙測 (GP3 無回傳?)");
        }
    }
}
