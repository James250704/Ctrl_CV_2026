// ============================================================================
// Hub — 中央轉發節點 (RP2350) — 雙核心閉環設計
//
// 根據 CLAUDE.md 拓撲：Jetson ←USB→ Hub ←PIO UART→ Slaves
//
// 兩顆核心各自形成獨立的閉環系統：
//
//   Core 0 (loop):  自動駕駛與路由大腦
//     - USB CDC (Serial) 收發 Jetson COBS 控制幀
//     - PIO UART TX 轉發控制命令至 Slaves
//     - PIO UART RX 接收 Slaves 回傳的 COBS 遙測幀
//     - Jetson 斷線時 (逾時 500ms)，讀取 Core 1 的 RC 數據作為備援
//     - 所有 I/O（USB + PIO TX/RX）集中在 Core 0，簡化同步模型
//
//   Core 1 (loop1): 安全守門員與手動介入
//     - 專職 crsf.update() — 高頻 CRSF 中斷解包 + Failsafe 計時器
//     - 只管遙控器，絕不漏字，Failsafe 判定極度精準
//     - 解完 -100~100 資料後，寫入全域變數供 Core 0 讀取
//
//  核心之間如何協同：
//    - Core 1 持續寫入 rc_channels[] / rc_failsafe (volatile 共用變數)
//    - Core 0 在其 loop() 中以 memcpy 低代價讀取
//    - 無鎖、無 IRQ、無跨核 handshake — 各自獨立閉環
//
// 控制邏輯：自動為主、遙控為輔
//    - Jetson 正常發送 → Core 0 直接轉發至 slaves (自動模式)
//    - Jetson 逾時 (>500ms) → Core 0 改用 Core 1 的遙控數據 (備援)
//    - 遙控也斷線 (Failsafe) → 發送 E-Stop 廣播至所有 slaves
//    - RC CH6 = -100 → 強制 E-Stop (安全開關)
//
// 腳位分配：
//   USB    ←→ Jetson (CDC Serial)       — ChassisComms 綁定 &Serial
//   UART0  ←→ ELRS 接收機 (GP1 RX)       — CRSF @ 420000 baud
//   PIO SM0 → Move slave (GP2 TX)        — ChassisComms 綁定 moveUart
//   PIO SM0 ← Move slave (GP3 RX)        — 接收 Move 遙測 COBS 幀
//   PIO SM1 → Shoot slave (GP4 TX)       — ChassisComms 綁定 shootUart
//   PIO SM1 ← Shoot slave (GP5 RX)       — 接收 Shoot 遙測 COBS 幀
//   PIO SM2 → Storage slave (GP6 TX)     — ChassisComms 綁定 storageUart
//   PIO SM2 ← Storage slave (GP7 RX)     — 接收 Storage 遙測 COBS 幀
//   UART1  → 除錯 (GP4 TX, GP5 RX)       — CLAUDE.md §UART 腳位, 115200 baud
// 注意：GP4/GP5 在 UART1 除錯與 PIO SM1 (Shoot) 之間衝突，須在硬體層面確保
//       不會同時啟用。當前預設 PIO SM1 優先於 UART1。
// ============================================================================
#include "ChassisComms.h"
#include <CrsfReceiver.h>
#include <Arduino.h>
#include <SerialPIO.h>

// ============================================================================
// 跨核心共用變數 — Core 1 寫入，Core 0 讀取
// RP2350 SCU 保證 cache coherency，volatile 防止編譯器過度優化
// ============================================================================
static volatile int16_t  rc_channels[10]; // -100 ~ 100，由 Core 1 更新
static volatile bool     rc_failsafe;     // true = 遙控器斷線

// ============================================================================
// PIO UART 硬體分配 (Hub 端)
// 根據 CLAUDE.md 規格，GP2~GP7 分配如下：
//
//   Slave     TX (Hub→Slave)  RX (Hub←Slave)
//   Move      GP2             GP3
//   Shoot     GP4             GP5
//   Storage   GP6             GP7
//
// 交叉連接：Hub PIO TX → Slave UART0 RX(GP1)，Slave UART0 TX(GP0) → Hub PIO RX
//   Move：    Hub SM0 TX(GP2) → Move RX(GP1)， Move TX(GP0) → Hub SM0 RX(GP3)
//   Shoot：   Hub SM1 TX(GP4) → Shoot RX(GP1)， Shoot TX(GP0) → Hub SM1 RX(GP5)
//   Storage： Hub SM2 TX(GP6) → Storage RX(GP1)， Storage TX(GP0) → Hub SM2 RX(GP7)
// ============================================================================
static constexpr uint8_t PIN_MOVE_TX    = 2;
static constexpr uint8_t PIN_MOVE_RX    = 3;
static constexpr uint8_t PIN_SHOOT_TX   = 4;
static constexpr uint8_t PIN_SHOOT_RX   = 5;
static constexpr uint8_t PIN_STORAGE_TX = 6;
static constexpr uint8_t PIN_STORAGE_RX = 7;

// earlephilhower SerialPIO：每個 slave 一個實例，各自使用獨立的 PIO SM。
// SerialPIO(txPin, rxPin, fifoSize)
static SerialPIO moveUart(PIN_MOVE_TX, PIN_MOVE_RX);       // TX=GP2, RX=GP3
static SerialPIO shootUart(PIN_SHOOT_TX, PIN_SHOOT_RX);    // TX=GP4, RX=GP5
static SerialPIO storageUart(PIN_STORAGE_TX, PIN_STORAGE_RX); // TX=GP6, RX=GP7

// ============================================================================
// PIO UART RX 接收緩衝區 — 每個 slave 獨立靜態緩衝區
// 用於暫存 Slave 回傳的 COBS 遙測幀，直到遇到 0x00 終止符後解碼。
// ============================================================================
static constexpr uint16_t PIO_RX_BUF_SIZE = 512;

struct PioRxBuffer {
    uint8_t buf[PIO_RX_BUF_SIZE];
    uint16_t len;
};

static PioRxBuffer moveRxBuf;
static PioRxBuffer shootRxBuf;
static PioRxBuffer storageRxBuf;

// ============================================================================
// 輔助函式：處理 PIO UART RX 收到的位元組
//
// 從指定 SerialPIO 讀取可用位元組，暫存至對應的 PioRxBuffer。
// 遇到 0x00 終止符時進行 COBS 解碼 + CRC 驗證，並在目前階段
// 透過 Serial.printf 輸出解碼結果供開發用。
// ============================================================================
static void processPioRx(SerialPIO& uart, PioRxBuffer& rxBuf, const char* name) {
    while (uart.available() > 0) {
        uint8_t byte = static_cast<uint8_t>(uart.read());

        if (byte == 0x00) {
            if (rxBuf.len == 0) {
                // 空幀或遺漏前一個終止符，跳過
                continue;
            }

            // COBS 解碼：rxBuf.buf[0..len-1] 包含 COBS 編碼資料
            // (含末尾的 0x00 終止符已排除，所以 buf 內容是去除最後 0x00 的編碼資料)
            uint8_t decoded[256];
            size_t decodedLen = 0;

            // COBS 解碼實作
            const uint8_t* src = rxBuf.buf;
            size_t remaining = rxBuf.len;
            bool decodeOk = false;

            if (remaining > 0 && remaining <= 254) {
                decodedLen = 0;
                decodeOk = true;

                while (remaining > 0) {
                    uint8_t code = *src++;
                    remaining--;

                    // 複製 code-1 個非零位元組
                    for (uint8_t i = 1; i < code && remaining > 0; i++) {
                        if (decodedLen >= sizeof(decoded)) {
                            decodeOk = false;
                            break;
                        }
                        uint8_t val = *src++;
                        remaining--;
                        if (val == 0x00) {
                            // COBS 編碼不應包含 0x00（除了終止符）
                            decodeOk = false;
                            break;
                        }
                        decoded[decodedLen++] = val;
                    }
                    if (!decodeOk) break;

                    // 如果 code < 0xFF，則此處原本有 0x00（終止符位於最末，故中途遇到表示資料中有 0x00）
                    if (code < 0xFF) {
                        if (remaining == 0) {
                            // 正常結束：這是最後一個區塊
                            break;
                        }
                        // 在中間遇到 0x00，這是有效資料（原本是 0x00 被編碼）
                        if (decodedLen >= sizeof(decoded)) {
                            decodeOk = false;
                            break;
                        }
                        decoded[decodedLen++] = 0x00;
                    }
                }
            }

            if (decodeOk && decodedLen >= 6 + 2) { // 至少 sync(1)+msgType(1)+frameSeq(1)+targetID(1)+payloadLen(2)+crc16(2)
                // CRC 驗證
                uint16_t storedCrc;
                uint16_t payloadLen;
                uint8_t  msgType;
                uint8_t  frameSeq;
                uint8_t  targetID;

                // 從 decoded 中提取欄位
                // Header: sync(1), msgType(1), frameSeq(1), targetID(1), payloadLen(2), crc16(2)
                msgType    = decoded[1];
                frameSeq   = decoded[2];
                targetID   = decoded[3];
                payloadLen = static_cast<uint16_t>(decoded[4]) | (static_cast<uint16_t>(decoded[5]) << 8);
                storedCrc  = static_cast<uint16_t>(decoded[decodedLen - 2]) | (static_cast<uint16_t>(decoded[decodedLen - 1]) << 8);

                uint8_t crcInput[decodedLen - 2];
                memcpy(crcInput, decoded, decodedLen - 2);
                uint16_t calcCrc = SignalUtils::calculateCrc(crcInput, decodedLen - 2);

                if (calcCrc == storedCrc) {
                    // CRC 驗證通過
                    Serial.printf("[PIO RX][%s] msgType=%d, frameSeq=%d, targetID=%d, payloadLen=%d, total=%zu bytes\n",
                                  name, msgType, frameSeq, targetID, payloadLen, decodedLen);
                } else {
                    Serial.printf("[PIO RX][%s] CRC mismatch (calc=0x%04X, stored=0x%04X), %zu bytes\n",
                                  name, calcCrc, storedCrc, decodedLen);
                }
            } else {
                Serial.printf("[PIO RX][%s] decode fail or too short: bufLen=%u, decodedLen=%zu\n",
                              name, rxBuf.len, decodedLen);
            }

            rxBuf.len = 0; // 重置緩衝區
        } else {
            // 一般位元組：寫入緩衝區
            if (rxBuf.len < PIO_RX_BUF_SIZE) {
                rxBuf.buf[rxBuf.len++] = byte;
            } else {
                // 緩衝區溢出：丟棄並重置
                rxBuf.len = 0;
                Serial.printf("[PIO RX][%s] buffer overflow, resetting\n", name);
            }
        }
    }
}

// ============================================================================
// ChassisComms 通用協議實例
//
// ChassisComms 是綁定到任意 Stream 的 COBS 協議庫：
//   - jetsonComms ←→ &Serial (USB CDC, Hub→Jetson 雙向，有心跳)
//   - moveComms / shootComms / storageComms → PIO UART TX (僅發送，無心跳)
//
// hbIntervalUs=0 停用心跳 — 因為 Hub→Slaves 方向為主機發送控制幀，
// 不需要定期心跳（Slave 端已有指令逾時 100ms E-Stop 保護）。
// ============================================================================
static ChassisComms jetsonComms(&Serial, TargetID::HUB);
static ChassisComms moveComms(&moveUart, TargetID::MOVE,    0);
static ChassisComms shootComms(&shootUart, TargetID::SHOOT,  0);
static ChassisComms storageComms(&storageUart, TargetID::STORAGE, 0);

// ============================================================================
// 遙控接收機 — Core 1 專用，使用 UART0 (GP1 RX) @ 420000 baud
// 因 Jetson 改用 USB CDC 通訊，UART0 已空出給 CRSF 使用。
//
// 接線：ELRS 接收機 TX → Hub GP1 (UART0 RX)
//       需在 ELRS 接收機設定 serial_inverted = off
// ============================================================================
static CrsfReceiver crsf;

// ============================================================================
// Core 0 內部狀態 — Jetson 連線追蹤 & RC 備援序號
// ============================================================================
static uint64_t lastJetsonFrameUs = 0;  // 最後收到 Jetson 有效封包的時間
static uint8_t  rcFallbackSeq     = 0;  // RC 備援轉發用的 frameSeq
static uint64_t lastRcForwardUs   = 0;  // RC 備援最後轉發時間

// ============================================================================
// 轉發輔助函式：對所有 slaves 廣播一個 ControlPacket
// 主要用於 E-Stop 廣播（targetID=BROADCAST + msgType=ESTOP）
// 一般控制不使用此函式（各 Slave 有各自的 targetID 和 payload）
// ============================================================================
static void forwardAllSlaves(const ControlPacket& pkt) {
    moveComms.sendPacket(pkt);
    shootComms.sendPacket(pkt);
    storageComms.sendPacket(pkt);
}

// ============================================================================
// RC 通道分配（ELRS CRSF 協定，CH1~CH10，值域 -100~100）
//
//   通道編號 (1-indexed) → rc_channels[index] (0-indexed)
//
//   Move:
//     CH1 (Roll)    → rc_channels[0] → vx       左右平移
//     CH2 (Pitch)   → rc_channels[1] → vy       前後移動
//     CH4 (Yaw)     → rc_channels[3] → omega    旋轉
//
//   Shoot:
//     CH3 (Throttle) → rc_channels[2] → upSpeed   上射速度
//     CH5            → rc_channels[4] → downSpeed 下射速度
//     CH7            → rc_channels[6] → pitch     俯仰
//     CH8            → rc_channels[7] → row       旋轉
//
//   Storage:
//     CH9            → rc_channels[8]  → motorChannel  PCA9685 通道 0~15
//     CH6 (AUX2)     → rc_channels[5]  → sw + E-Stop   0/1，-100 時所有歸零
//     CH10           → rc_channels[9]  → trigger       0/1（> 50 視為 1）
// ============================================================================

// 死區濾波：-10~10 → 0
static int16_t applyDeadband(int16_t val) {
    if (val > -10 && val < 10) return 0;
    return val;
}

// 將 CH8 (-100~100) 映射到 PCA9685 通道 0~15
static uint8_t channelToPca9685(int16_t ch) {
    int mapped = (static_cast<int>(ch) + 100) * 15 / 200; // -100→0, 100→15
    if (mapped < 0)  return 0;
    if (mapped > 15) return 15;
    return static_cast<uint8_t>(mapped);
}

// 將遙控通道值映射為 int8（含死區 + 限幅）
static int8_t rcToInt8(int16_t ch) {
    return static_cast<int8_t>(constrain(applyDeadband(ch), -100, 100));
}

// 將遙控通道值映射為 boolean（> 50 視為 true）
static bool rcToBool(int16_t ch) {
    return ch > 50;
}

// ============================================================================
// RC → Move ControlPacket
// ============================================================================
static ControlPacket rcToMovePacket() {
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = rcFallbackSeq++;
    pkt.targetID   = TargetID::MOVE;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    pkt.move.vx    = rcToInt8(rc_channels[0]);   // CH1 → vx
    pkt.move.vy    = rcToInt8(rc_channels[1]);   // CH2 → vy
    pkt.move.omega = rcToInt8(rc_channels[3]);   // CH4 → omega

    // CH6 (安全開關) = -100 時強制歸零
    if (rc_channels[5] < 0) {
        pkt.move.vx = pkt.move.vy = pkt.move.omega = 0;
    }

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
                    reinterpret_cast<uint8_t*>(&pkt),
                    sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// RC → Shoot ControlPacket
// ============================================================================
static ControlPacket rcToShootPacket() {
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = rcFallbackSeq++;
    pkt.targetID   = TargetID::SHOOT;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    pkt.shoot.upSpeed   = rcToInt8(rc_channels[2]);  // CH3 → upSpeed
    pkt.shoot.downSpeed = rcToInt8(rc_channels[4]);  // CH5 → downSpeed
    pkt.shoot.pitch     = rcToInt8(rc_channels[6]);  // CH7 → pitch
    pkt.shoot.row       = rcToInt8(rc_channels[7]);  // CH8 → row

    // CH6 (安全開關) = -100 時強制歸零
    if (rc_channels[5] < 0) {
        pkt.shoot.upSpeed = pkt.shoot.downSpeed = 0;
        pkt.shoot.pitch = pkt.shoot.row = 0;
    }

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
                    reinterpret_cast<uint8_t*>(&pkt),
                    sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// RC → Storage ControlPacket
// ============================================================================
static ControlPacket rcToStoragePacket() {
    ControlPacket pkt = {};
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = rcFallbackSeq++;
    pkt.targetID   = TargetID::STORAGE;
    pkt.payloadLen = sizeof(ControlPacket) - 6 - 2;

    pkt.storage.motorChannel = channelToPca9685(rc_channels[8]);  // CH9 → 通道
    pkt.storage.sw           = rcToBool(rc_channels[5]) ? 1 : 0;  // CH6 → switch
    pkt.storage.trigger      = rcToBool(rc_channels[9]) ? 1 : 0; // CH10 → trigger

    // CH6 (安全開關) = -100 時強制關閉
    if (rc_channels[5] < 0) {
        pkt.storage.sw = 0;
        pkt.storage.trigger = 0;
    }

    pkt.crc16 = 0;
    pkt.crc16 = SignalUtils::calculateCrc(
                    reinterpret_cast<uint8_t*>(&pkt),
                    sizeof(ControlPacket) - 2);
    return pkt;
}

// ============================================================================
// Core 0：Jetson 幀回呼
//
// jetsonComms 從 USB CDC 解出有效 ControlPacket 後觸發。
// Core 0 當場根據 targetID 呼叫對應 slave 的 ChassisComms.sendPacket()，
// 一條龍處理，完全不需跨核心傳遞。
// ============================================================================
static void onJetsonFrame(const ControlPacket& pkt) {
    lastJetsonFrameUs = time_us_64();

    switch (pkt.targetID) {
        case TargetID::MOVE:
            moveComms.sendPacket(pkt);
            break;
        case TargetID::SHOOT:
            shootComms.sendPacket(pkt);
            break;
        case TargetID::STORAGE:
            storageComms.sendPacket(pkt);
            break;
        case TargetID::BROADCAST:
            forwardAllSlaves(pkt);
            break;
        case TargetID::HUB:
        default:
            break;
    }
}

// ============================================================================
// Core 0：RC 備援轉發 — 當 Jetson 逾時時由 loop() 呼叫
// 以 ~100Hz 頻率將遙控數據轉發至所有 slave
// ============================================================================
static void tryRcFallback() {
    uint64_t now = time_us_64();

    // 節流至 ~100Hz 避免 flooding
    if (now - lastRcForwardUs < 10000ULL) // 10ms
        return;
    lastRcForwardUs = now;

    // 如果遙控也斷線 → E-Stop 廣播
    if (rc_failsafe) {
        ControlPacket estop = {};
        estop.sync       = 0xA5;
        estop.msgType    = MsgType::ESTOP;
        estop.frameSeq   = rcFallbackSeq++;
        estop.targetID   = TargetID::BROADCAST;
        estop.payloadLen = sizeof(ControlPacket) - 6 - 2;
        estop.crc16      = 0;
        estop.crc16 = SignalUtils::calculateCrc(
                          reinterpret_cast<uint8_t*>(&estop),
                          sizeof(ControlPacket) - 2);
        forwardAllSlaves(estop);
        return;
    }

    // 遙控正常 → 分別轉發控制幀給三個 Slave
    moveComms.sendPacket(rcToMovePacket());
    shootComms.sendPacket(rcToShootPacket());
    storageComms.sendPacket(rcToStoragePacket());
}

// ============================================================================
// Core 0：setup()
// ============================================================================
void setup() {
    Serial.begin(921600);  // Jetson USB CDC

    // Jetson USB CDC — ChassisCombs 綁定 &Serial, 30ms 心跳往 Jetson
    jetsonComms.begin();
    jetsonComms.setFrameCallback(onJetsonFrame);

    // Slave PIO UART — ChassisComms 綁定 SerialPIO，無心跳 (hbIntervalUs=0)
    moveUart.begin(RobotConfig::PIO_BAUDRATE);
    moveComms.begin();
    shootUart.begin(RobotConfig::PIO_BAUDRATE);
    shootComms.begin();
    storageUart.begin(RobotConfig::PIO_BAUDRATE);
    storageComms.begin();

    // 初始化接收緩衝區長度
    moveRxBuf.len    = 0;
    shootRxBuf.len   = 0;
    storageRxBuf.len = 0;

    // 初始化 Jetson 逾時追蹤
    lastJetsonFrameUs = time_us_64();
}

// ============================================================================
// Core 0: loop() — 自動駕駛與路由大腦 + PIO RX 接收
//
// 1. jetsonComms.update() → USB CDC 讀取 + COBS 解包 + 回呼轉發 + 30ms 心跳
// 2. PIO UART RX 輪詢 — 接收各 Slave 回傳的 COBS 遙測幀
// 3. 如果 Jetson 逾時 >500ms → 切換至 RC 備援
// ============================================================================
void loop() {
    jetsonComms.update();

    // 輪詢各 PIO UART RX，接收 Slave 回傳的 COBS 遙測幀
    processPioRx(moveUart,    moveRxBuf,    "Move");
    processPioRx(shootUart,   shootRxBuf,   "Shoot");
    processPioRx(storageUart, storageRxBuf, "Storage");

    uint64_t now = time_us_64();
    if (now - lastJetsonFrameUs > RobotConfig::HEARTBEAT_TIMEOUT_US) {
        // Jetson 斷線 → 啟用遙控備援
        tryRcFallback();
    }

    delayMicroseconds(20);
}

// ============================================================================
// Core 1: setup1() — 初始化 CRSF 遙控接收機
//
// CRSF 使用 UART0 (GP1 RX, 420000 baud)。
// 因 Jetson 改用 USB CDC，UART0 已空出給 CRSF 使用。
// ELRS 接收機需設為非反向輸出 (serial_inverted = off)。
// ============================================================================
void setup1() {
    crsf.begin(0, 1, 420000);
}

// ============================================================================
// Core 1: loop1() — 安全守門員與手動介入
//
// 專職 crsf.update() 高頻解包，什麼都不管，只專心抓遙控器。
// 解完後寫入全域 rc_channels / rc_failsafe，供 Core 0 低代價讀取。
// ============================================================================
void loop1() {
    while (true) {
        crsf.update();

        // 逐元素寫入全域變數 (避免 volatile pointer 傳入 memcpy 的編譯錯誤)
        // RP2350 SCU 保證 cache coherency，Core 0 讀取時可見最新值
        int16_t ch[10];
        crsf.getChannels(ch);
        for (int i = 0; i < 10; i++) {
            rc_channels[i] = ch[i];
        }
        rc_failsafe = crsf.isFailsafe();
    }
}
