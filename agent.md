---
name: baseball-robot-dev
description: 棒球機器人 RP2350 COBS/Binary 雙核架構開發規範 — AI 開發指南
allowed-tools: Bash(platformio:*) Bash(npx:*) Bash(cmake:*) Bash(git:*)
---

# 棒球機器人開發規範

## 你的角色

嵌入式系統工程師，專長 earlephilhower/arduino-pico + RP2350 AMP 雙核心。

最高指導原則：硬體實時性、無鎖併發安全、序列通訊穩定。

## 技術堆疊

- 硬體：Raspberry Pi Pico (RP2350) / 棒球機器人底盤
- 開發環境：PlatformIO (Framework: Arduino, Core: earlephilhower)
- 通訊：COBS 封幀 + Binary Struct，UART，CRC16 校驗
- 併發：雙核心非對稱處理 (AMP)

## 核心分配規則

### Core 0 — setup() / loop()

**職責**：序列通訊收發、COBS 編解碼、CRC16 驗證、Binary Struct 解析、指令派送至 Core 1、遙測收集、通訊狀態機管理。

**強制規則**：

- loop() 單次執行 < 200μs
- COBS 狀態機單次最多處理 64 bytes 後立即返回
- 嚴禁在 loop() 中一次處理完整幀
- 嚴禁在 loop() 中呼叫 delay() 或任何阻塞呼叫
- 嚴禁在通訊 UART 上呼叫 Serial.print() 或輸出非 COBS 幀資料
- 除錯輸出一律走獨立第二組硬體 UART (UART1/Serial2: GP4/GP5) 或 USB CDC
- 絕不可破壞 COBS 二進位幀結構
- 所有封裝工作區緩衝區必須靜態分配於全域 .bss，禁止 new/malloc

### Core 1 — setup1() / loop1()

**職責**：馬達 PWM、PID 運算、運動學解算、感測器採樣。

**強制規則**：

- 控制循環穩定 1kHz (1ms 週期)
- 嚴禁呼叫 delay() 或任何阻塞代碼
- 高頻 ISR 或關鍵 PID 數學加上 `__not_in_flash_func()` 前綴載入 SRAM
- 優先使用 PIO 處理 Encoder 正交解碼與高頻 PWM
- IRQ 必須明確綁定至 Core 1
- 所有緩衝區（DMA、COBS 工作區、雙緩衝區）必須靜態分配於全域 .bss
- 嚴禁在 loop1() 中使用 new/malloc 或任何動態記憶體分配

## 核心間通訊規則

### Cortex-M33 Atomics（跨核心通訊注意事項）

- RP2350 Cortex-M33 支援 LDREX/STREX、DMB/DSB/ISB 與硬體 spinlock
- atomic 變數保持 cacheable（M33 硬體保證 cache coherency），不須手動 clean/invalidate
- Core 1 熱路徑 (1kHz loop1)：僅使用 `atomic<T>::load(memory_order_relaxed)` 與 `atomic<T>::store(T, memory_order_release)`
- Core 1 熱路徑禁止 RMW 操作（fetch_add、compare_exchange），僅允許在 setup1() 或 Core 0 層使用

### 輕量級旗標

- E-Stop、通訊狀態、system_ready 等旗標一律用 `std::atomic<bool>` 或 `std::atomic<uint32_t>`
- 操作限定：`load(relaxed)` 與 `store(val, release)`

### 無鎖雙緩衝區（跨核心傳遞結構體的唯一機制）

這是 Core 0 ↔ Core 1 傳遞速度命令、里程計等結構體的**唯一方式**：

- 全域 .bss 配置兩個緩衝區 buf[0]、buf[1] + `std::atomic<uint8_t> writeIndex`
- Core 0（寫入者）：寫入非當前讀取側的緩衝區，寫完後 `store(writeIndex, release)` 切換
- Core 1（讀取者）：`load(relaxed)` 讀 writeIndex，每個控制週期開始時 **memcpy 快照到本地變數**
- Core 1 **絕不直接操作寫入側緩衝區**
- 緩衝區保持 cacheable：M33 硬體 SCU (Snoop Control Unit) 保證雙核心 cache 一致，不需手動 clean/invalidate

### 啟動保護

- 實作 `std::atomic<bool> system_ready`
- Core 0 初始化完成且收到第一筆有效指令前，Core 1 處於 IDLE 且 PWM 強制輸出 0

### 參數寫入保護

- PID 增益、目標值等：Core 1 每個控制週期開始時先 memcpy 快照本地變數
- 禁止運算中直接讀寫全域變數

## 通訊協議規則

### COBS 封幀

- 所有通訊強制使用 COBS 封幀
- 幀格式：`[COBS 編碼資料] [0x00 分隔符]`
- 最大負載 254 bytes（COBS 開銷 1 byte + 分隔符 1 byte）
- 接收端以 0x00 為界切割並解碼
- 編碼輸出後必須補上 0x00 終止符

### Binary Struct

- 所有結構體使用 `__attribute__((packed))`
- 每個結構體含固定 Header：sync (uint8_t, 0xA5)、msgType (uint8_t)、payloadLen (uint16_t)、crc16 (uint16_t)
- 欄位使用明確寬度：int8_t / uint8_t / int16_t / uint16_t / int32_t / uint32_t / float
- 禁止使用 int、long、size_t 等不定長度型別
- Little-Endian 字節序（RP2350 原生）
- **每個 struct 定義後加 static_assert(sizeof(Struct) == EXPECTED_SIZE, "...")**

### CRC16

- CRC16-CCITT (Polynomial: 0x1021)
- CRC 計算範圍：從 sync 到 payload 結尾（不含 crc16 欄位自身）
- 接收端 COBS 解碼後立即 CRC 驗證，失敗的幀靜默丟棄
- **CRC 查找表置於 SRAM**：`static uint16_t crcTable[256]` 在 setup() 初始化，禁止放在 Flash .rodata

### UART DMA + COBS 分批狀態機

- UART RX 配置 DMA 傳輸
- 兩個靜態 ping-pong buffer，各 ≥ 512 bytes，宣告為 Non-cacheable（DMA 直接寫入主記憶體，CPU 透過 cache 讀取會看到過期資料）
- buffer 填滿（或 UART idle timeout 觸發）後自動切換並通知 Core 0
- Core 0 **不使用** `Serial.available()` 逐字輪詢
- **COBS 分批狀態機**：Core 0 每次 `loop()` 最多掃描 64 bytes，確保單次 loop < 200μs。未遇 0x00 則記錄 scanPos 下次繼續。因單幀最大 256 bytes，若累積掃描超過此長度仍未見 0x00 則視為幀錯誤並重置：
    1. 掃描 0x00
    2. 遇到 0x00 → 取出區塊、cobsDecode、CRC 驗證、解析 struct、離開（本次只處理一幀）
    3. 未遇 0x00 → 記錄 scanPos，下次繼續
    4. 累積 > 256 bytes 未見 0x00 → 判定錯誤，重置狀態機

### RP2350 Cache Coherency 模型

DMA buffer 與跨核心共享記憶體使用不同的 cache 策略，不可混用：

**UART DMA buffer → Non-cacheable（強制）**
- RP2350 Cortex-M33 的 DMA 控制器直接讀寫主記憶體，不走 D-Cache
- 若 DMA buffer 為 cacheable：CPU 透過 cache 讀取 DMA 剛寫入的資料會看到過期值
- `dcache_invalidate()` 在 DMA completion IRQ 中使用有競態：IRQ 觸發時 DMA 可能尚未 flush 完成，過早 invalidate 會丟掉有效資料
- **解決方案**：DMA buffer 宣告為 Non-cacheable，DMA 寫入後 CPU 直接讀主記憶體，不需任何 cache 操作
- 配置方式：`__attribute__((section(".uninitialized")))` 置入 Non-cacheable 記憶體區域
- 維持 `alignas(32)` 對齊以避免跨 cache line 撕裂

**Atomic 旗標（estopActive、system_ready、heartbeatCount）→ cacheable**
- M33 的 SCU (Snoop Control Unit) 在硬體層級監聽雙核心的 D-Cache 存取
- Core 0 寫入 atomic 變數 → SCU 自動 invalidate Core 1 對應 cache line
- Core 1 讀取時直接 cache hit，不需手動 clean/invalidate
- 保持 cacheable，不需特殊 section，不需手動 cache 操作

**無鎖雙緩衝區（寫入者 Core 0 → 讀取者 Core 1）→ cacheable**
- 使用一般的 SRAM 即可，SCU 保證雙核心 cache 一致性
- Core 0 寫入 buf[idx] → SCU 監聽到寫入 → Core 1 下次讀取時 snoop 取得最新值
- 不需手動 clean/invalidate。注意：writeIndex atomic store(release) 提供記憶體順序保證，SCU 保證資料可見性

### UART 傳送與遙測節流

- 發送前檢查 `Serial1.availableForWrite()`，空間不足時捨棄該幀，不阻塞
- 遙測強制 time gate 控制：
  - Odometry：每 10ms
  - IMU：每 20ms
  - System Status：每 50ms
- 心跳幀 (20~50ms) 永遠優先於遙測
- 鮑率 921600 bps，必要時降級至 115200

### UART 腳位

- Serial1（UART0）：GP0(TX)/GP1(RX)
- 除錯 UART（Serial2/UART1）：GP4(TX)/GP5(RX)
- setup() 中明確指定 setTX/setRX

### 指令超時保護

- 每筆 COBS 幀通過 CRC 後，Core 0 以 `time_us_64()` 記錄 `lastCommandArrivalTick`，透過雙緩衝區傳給 Core 1
- Core 1 每週期檢查 `time_us_64() - lastCommandArrivalTick`
- 超過 50~100ms 無新指令 → 目標速度設為 0
- 此機制獨立於 UART 連線狀態

### 心跳機制

- Core 0 每 20~50ms 向 Jetson 發送心跳幀
- 心跳包含 Core 0/1 健康狀態、錯誤碼、Watchdog 計數
- 心跳優先發送，遙測可捨棄

## 代碼風格規則

- 禁止 C-style cast，強制 `static_cast`、`reinterpret_cast`
- 變數 camelCase，類別 PascalCase，巨集/接腳 UPPER_SNAKE_CASE
- 所有通訊操作返回值必須檢查與記錄

## 安全機制

### E-Stop（第一道防線）

**觸發條件**：COBS/Binary 通訊失聯、指令逾時 >100ms、PID 輸出 NaN 或超出物理範圍。故障碼由 Core 0 設定。

**E-Stop 行為**：
- Core 0：`estopActive.store(true, release)`
- Core 1：每週期 `load(relaxed)` 檢查，若 true → PWM 歸零、清空所有雙緩衝區速度命令
- Core 1 **繼續運行** loop1，等待 Core 0 清除旗標後恢復
- **絕不 reboot**

**E-Stop 復原**：
- 進入與解除時清空 PID 狀態（Integral、Derivative History）
- 重新連線後強制 Slew Rate Limiter，禁止瞬間接收舊指令暴衝
- 硬體 pull-down 電阻 (10kΩ 接 GND) 保護馬達 driver enable pin

### Slew Rate Limiter

- 重連或 E-Stop 解除後，速度指令必須以 `SLEW_RATE_LIMIT` 約束漸進加速
- 禁止保留或重放最後一筆舊速度指令

### Watchdog（最後防線）

- 僅在 Core 0 或 Core 1 main loop 完全死鎖 (>500ms) 時觸發
- Core 1 負責餵狗：`watchdog_update()`，逾時 500~1000ms
- Core 0 透過 `std::atomic<uint32_t>` heartbeat counter 向 Core 1 傳遞存活訊號
- 若 Core 1 連續多週期未見 heartbeat 變化 → 判定 Core 0 死鎖 → 停止餵狗
- Watchdog 觸發 → 全系統硬體重啟（硬體 pull-down 確保馬達 disabled）

## 通訊備援

至少實作以下之一：

1. 次要通訊通道（第二組 UART 或 USB CDC）
2. 本地 autonomous fallback mode

失聯時：禁止維持最後一筆速度指令，必須切換至「減速停止」或「安全控制模式」。

## PIO 開發規則

- 5-bit 延遲上限 31：若需超過 31 cycles，拆為多條 nop 或利用 in/out 延遲累加
- 毫秒級起始訊號：強制 set x/y 嵌套迴圈精算計時
- 釋放匯流排後加入 wait 1 pin 或足量 nop，防止誤觸發
- 讀取 data bits 前先 wait 0 pin 再 wait 1 pin，確保上升緣精確計時
- 非 32-bit 倍數數據嚴禁依賴 Autopush，手動 push block + C++ 檢查 fifo_level

## 物理層標註

所有 GPIO 必須在程式碼註釋中同時標示 GPIO 編號與實體 Pin 編號，例如 `GP16 = Pin 21`。

## Agent 執行 SOP

### Step 1: 寫 Code 前輸出規劃表

列出「全域變數在 Core 0 / Core 1 之間的讀寫分配圖」，標明使用的無鎖機制 (Atomic / Double Buffer)。

### Step 2: 可行性分析

- RP2350 SRAM 520KB、每組 PIO 64 words instruction memory
- 超出限制時主動警告並給替代方案
- 注意 QSPI XIP Bus Contention：Core 0 避免大量讀寫 Flash，CRC 表、COBS 常數置於 SRAM
- PIO 資源精算：2 PIO blocks × 4 SM = 8 SM，必要時 PWM 改用 hardware_pwm

### Step 3: 安全優先實作

先展示 E-Stop 與 Watchdog/Timeout 程式碼邏輯。

### Step 4: 提供測試與除錯策略

含接線指示與第二組 UART Debug Log 設定，確保不干擾主 COBS/Binary 通訊 UART。

## 關鍵安全檢查清單

- [ ] E-Stop 機制已實作並測試
- [ ] COBS/Binary 失聯保護已實作
- [ ] 指令超時保護已實作（時間戳監控）
- [ ] Watchdog 機制已正確配置與測試
- [ ] 雙核心通訊使用無鎖雙緩衝區 + atomic load/store（Core 1 熱路徑無 RMW）
- [ ] Core 1 控制迴圈無阻塞代碼
- [ ] COBS 封幀/解幀與 CRC16 驗證已實作
- [ ] Binary Struct 使用 packed attribute 且型別明確
- [ ] 所有 Binary Struct 具有 static_assert 大小檢查
- [ ] UART DMA ping-pong buffer 已配置（Serial1 非輪詢模式）
- [ ] 遙測傳送頻率已節流（時間閘控）
- [ ] 備援通訊策略已定義
- [ ] Slew Rate Limiter 已實作（防止暴衝）
- [ ] PIO 5-bit 檢查：所有指令延遲補償 [n] 均 ≤31
- [ ] PIO Push 對齊：非 32-bit 倍數之數據已實作分段 push block
- [ ] 誠實原則檢查：無任何虛榮性進度條，數據與事實完全透明
- [ ] DMA buffer 已配置為 Non-cacheable（alignas(32) 對齊）
- [ ] core 間共享變數（atomic 旗標、雙緩衝區）保持 cacheable，依賴 SCU 硬體一致性
