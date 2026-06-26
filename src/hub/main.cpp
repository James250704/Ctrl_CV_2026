// ============================================================================
// Hub — 中央轉發節點 (RP2350) — 雙核心閉環設計
//
// 根據 agent.md 拓撲：Jetson ←USB→ Hub ←PIO UART→ Slaves
//
// 兩顆核心各自形成獨立的閉環系統：
//
//   Core 0 (loop):  自動駕駛與路由大腦
//     - 專心伺候 Jetson USB CDC (ChassisComms COBS 解包)
//     - Jetson 送來控制命令時，Core 0 當場解包、當場呼叫 PIO UART 轉發
//       一條龍處理，完全不需跨核心傳遞，效率最高
//     - Jetson 斷線時 (逾時 500ms)，讀取 Core 1 的 RC 數據作為備援
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
//    - RC CH5 = -100 → 強制 E-Stop (安全開關)
//
// 腳位分配：
//   USB    ←→ Jetson (CDC Serial)     — ChassisComms 綁定 &Serial
//   UART0  ←→ ELRS 接收機 (GP1 RX)     — CRSF @ 420000 baud
//   PIO SM0 → Move slave (GP6 TX)      — ChassisComms 綁定 moveUart
//   PIO SM1 → Shoot slave (GP8 TX)     — ChassisComms 綁定 shootUart
//   PIO SM2 → Storage slave (GP10 TX)  — ChassisComms 綁定 storageUart
//   UART1  → 除錯 (GP4 TX, GP5 RX)     — agent.md §UART 腳位, 115200 baud
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
// 根據 agent.md §UART 腳位 + 使用者指定腳位：
//
//   Slave     TX (Hub→Slave)  RX (Hub←Slave)
//   Move      GP6             GP7
//   Shoot     GP8             GP9
//   Storage   GP10            GP11
//
// 交叉連接：Hub TX → Slave RX，Slave TX → Hub RX
//   Move：    GP6 → Move RX(GP5)， Move TX(GP4) → GP7
//   Shoot：   GP8 → Shoot RX(GP17)， Shoot TX(GP16) → GP9
//   Storage： GP10 → Storage RX(GP21)， Storage TX(GP20) → GP11
// ============================================================================
static constexpr uint8_t PIN_MOVE_TX    = 6;
static constexpr uint8_t PIN_SHOOT_TX   = 8;
static constexpr uint8_t PIN_STORAGE_TX = 10;

// earlephilhower SerialPIO：每個 slave 一個實例，各自使用獨立的 PIO SM。
static SerialPIO moveUart(0, 0, PIN_MOVE_TX);
static SerialPIO shootUart(0, 1, PIN_SHOOT_TX);
static SerialPIO storageUart(0, 2, PIN_STORAGE_TX);

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
// ============================================================================
static void forwardAllSlaves(const ControlPacket& pkt) {
    moveComms.sendPacket(pkt);
    shootComms.sendPacket(pkt);
    storageComms.sendPacket(pkt);
}

// ============================================================================
// 將遙控通道映射為 ControlPacket（Core 0 使用，RC 備援模式）
//
// 通道映射：
//   CH0 (Roll)  → vx    (左右平移)
//   CH1 (Pitch) → vy    (前後移動)
//   CH3 (Yaw)   → omega (旋轉)
//   CH5         → -100 = 安全開關切斷 (E-Stop)，+100 = 允許控制
// ============================================================================
static ControlPacket rcToControlPacket() {
    ControlPacket pkt;
    pkt.sync       = 0xA5;
    pkt.msgType    = MsgType::CONTROL;
    pkt.frameSeq   = rcFallbackSeq++;
    pkt.targetID   = TargetID::BROADCAST;
    pkt.payloadLen = sizeof(ControlPacket) - 6;

    // 遙控通道映射
    int16_t vx_raw    = rc_channels[0];
    int16_t vy_raw    = rc_channels[1];
    int16_t omega_raw = rc_channels[3];

    // 死區 -10~10 → 0，避免搖桿中立點抖動
    auto applyDeadband = [](int16_t val) -> int16_t {
        if (val > -10 && val < 10) return 0;
        return val;
    };

    pkt.vx    = static_cast<int8_t>(constrain(applyDeadband(vx_raw),    -100, 100));
    pkt.vy    = static_cast<int8_t>(constrain(applyDeadband(vy_raw),    -100, 100));
    pkt.omega = static_cast<int8_t>(constrain(applyDeadband(omega_raw), -100, 100));

    // 安全開關：CH5 = -100 時強制 FAILSAFE
    if (rc_channels[4] < 0) {
        pkt.mode = RobotMode::FAILSAFE;
        pkt.vx = pkt.vy = pkt.omega = 0;
    } else {
        pkt.mode = RobotMode::MANUAL;
    }

    uint64_t now = time_us_64();
    pkt.timestamp_hi = static_cast<uint32_t>(now >> 32);
    pkt.timestamp_lo = static_cast<uint32_t>(now & 0xFFFFFFFF);
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
        ControlPacket estop;
        estop.sync       = 0xA5;
        estop.msgType    = MsgType::ESTOP;
        estop.frameSeq   = rcFallbackSeq++;
        estop.targetID   = TargetID::BROADCAST;
        estop.payloadLen = sizeof(ControlPacket) - 6;
        estop.vx = estop.vy = estop.omega = 0;
        estop.mode = RobotMode::FAILSAFE;
        estop.timestamp_hi = static_cast<uint32_t>(now >> 32);
        estop.timestamp_lo = static_cast<uint32_t>(now & 0xFFFFFFFF);
        estop.crc16 = 0;
        estop.crc16 = SignalUtils::calculateCrc(
                          reinterpret_cast<uint8_t*>(&estop),
                          sizeof(ControlPacket) - 2);
        forwardAllSlaves(estop);
        return;
    }

    // 遙控正常 → 轉發控制幀
    ControlPacket pkt = rcToControlPacket();
    forwardAllSlaves(pkt);
}

// ============================================================================
// Core 0：setup()
// ============================================================================
void setup() {
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

    // 初始化 Jetson 逾時追蹤
    lastJetsonFrameUs = time_us_64();
}

// ============================================================================
// Core 0: loop() — 自動駕駛與路由大腦
//
// 1. jetsonComms.update() → USB CDC 讀取 + COBS 解包 + 回呼轉發 + 30ms 心跳
// 2. 如果 Jetson 逾時 >500ms → 切換至 RC 備援
// ============================================================================
void loop() {
    jetsonComms.update();

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
