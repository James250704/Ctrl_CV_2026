---
name: baseball-robot-dev
description: 棒球機器人 RP2350 — 拓撲明確的 Hub/Slave 雙核 COBS/PIO 開發規範
---

# 棒球機器人開發規範

## 系統拓撲

```
┌──────────┐   USB(COBS)    ┌──────────────────────────────────────┐
│  Jetson  │◄──────────────►│  Hub (RP2350)    中央轉發控制          │
│ (主電腦)  │ CDC ACM        │  - setup():    USB CDC 初始化         │
└──────────┘                │  - setup1():   各 Slave PIO UART 初始化 │
                             │  - loop():     與 Jetson USB 收發      │
                             │  - loop1():    與 Slaves PIO UART 收發 │
                             └──┬───────┬───────┬───────┘
                   PIO UART(COBS)│       │       │
                             ┌──┘       │       └──┐
                             ▼          ▼          ▼
                      ┌──────────┐ ┌──────────┐ ┌──────────┐
                      │  Move    │ │  Shoot   │ │ Storage  │
                      │ (RP2350) │ │ (RP2350) │ │ (RP2350) │
                      │ setup(): │ │ setup(): │ │ setup(): │
                      │ PIO UART │ │ PIO UART │ │ PIO UART │
                      │ setup1():│ │ setup1():│ │ setup1():│
                      │ 馬達初始化│ │ 馬達初始化│ │ 馬達初始化│
                      │ loop():  │ │ loop():  │ │ loop():  │
                      │ 通訊     │ │ 通訊     │ │ 通訊     │
                      │ loop1(): │ │ loop1(): │ │ loop1(): │
                      │ 馬達控制 │ │ 馬達控制 │ │ 馬達控制 │
                      └──────────┘ └──────────┘ └──────────┘
```

- **Jetson → Hub** : USB CDC ACM，COBS 封幀 + Binary struct + CRC16
- **Hub → Slaves** : PIO UART，COBS 封幀 + Binary struct + CRC16（每個 Slave 一條獨立的 PIO UART link）
- **Slave（move / shoot / storage）**：使用 **PIO UART** 收發 TX/RX 訊號，不用硬體 UART 外設
  - 好處：腳位任意指定、精確 bit timing、抗干擾能力優於中斷驅動的 UART
  - Hub 端也使用 PIO UART 與對應的 Slave 通訊，每個 Slave 使用一組獨立的 PIO 狀態機
- **開發框架**：PlatformIO + Earlephilhower Arduino-Pico（RP2350 支援）
- **雙核心啟動**：統一使用 `setup()` + `setup1()` 初始化，`loop()` + `loop1()` 主迴圈

## 核心分配

### Hub (RP2350)

| Core | 函式 | 職責 |
|------|------|------|
| Core 0 | `setup()` / `loop()` | USB CDC 初始化、與 Jetson 收發 COBS 幀 → 解析 → 轉發至對應 Slave |
| Core 1 | `setup1()` / `loop1()` | 各 Slave PIO UART 初始化、與 Slaves PIO UART 收發 COBS 幀 |

### Slaves — Move / Shoot / Storage (RP2350)

| Core | 函式 | 職責 |
|------|------|------|
| Core 0 | `setup()` / `loop()` | PIO UART 收發 COBS 幀、CRC 驗證、指令解析、通知 Core 1 |
| Core 1 | `setup1()` / `loop1()` | 馬達初始化、**由 Core 0 觸發**：收到有效控制幀 → 立即執行馬達控制 |

### 框架注意事項

- Earlephilhower Arduino-Pico 的雙核心啟動順序：`setup()` → `setup1()`（Core 0 → Core 1）
- `loop()` 和 `loop1()` 分別在 Core 0 和 Core 1 獨立運作
- 全域變數在雙核心間共享，需注意 atomic / memory_order 同步（參見核心間通訊章節）
- 禁止在 `setup1()` / `loop1()` 中使用 `Serial` 相關呼叫（USB CDC 綁定 Core 0）

## Slave 核心間同步機制（全車同步關鍵）

**原則：Slave Core 1 (`loop1()`) 不設獨立定時器迴圈，由 Core 0 (`loop()`) 收到有效控制幀後觸發執行。**

```
Core 0 (loop):
  1. PIO UART 收到 COBS 幀 → CRC 解析 → 驗證有效
  2. 寫入雙緩衝區 → atomic store release 通知 Core 1
  3. 發送 IRQ 或 spinloop 等待 Core 1 完成

Core 1 (loop1):
  1. 持續 spinloop 輪詢 Core 0 通知
  2. 收到通知 → memcpy 快照命令 → 執行馬達控制（PIO PWM）
  3. 完成後標示 done，回到輪詢
```

**同步流程：**
1. Jetson 送出控制幀 → Hub 接收 → Hub 轉發至所有 Slave
2. 各 Slave Core 0 收到、解析、CRC 驗證通過
3. Core 0 → Core 1 通知：「執行這個命令」
4. Core 1 執行 PIO 馬達控制，完成後回報
5. 所有 Slave 完成後，下一個控制週期開始

### 優點

- **全車硬同步**：所有馬達同時收到命令、同時執行，無累積時序漂移
- **無需獨立 timing**：Slave 不需自己維護 1kHz timer，依賴 Master（Jetson）的控制幀頻率

### 注意事項

- Slave Core 1 的執行時間必須小於控制幀間隔（典型 10~20ms），否則會漏幀
- 若長時間（>100ms）無新幀 → Core 1 E-Stop 觸發，PWM 歸零

## Core 0 通訊規則

- PIO UART RX 使用 IRQ + ring buffer（不輪詢、不 block）
- COBS 完整解一幀後才進行 CRC 驗證、派送指令
- Hub Core 0 使用 USB CDC ACM 收發（非 UART），同樣走 COBS + CRC
- 通訊專用 PIO UART — `Serial.print()` / `printf()` 走 USB CDC
- 所有工作緩衝區靜態分配於 .bss，禁止 new/malloc
- Hub 端的 PIO UART 腳位：每個 Slave 一組獨立的 PIO 狀態機 TX/RX

## Core 1 馬達控制規則

- 收到 Core 0 通知後立即執行一次控制循環（PIO PWM / Encoder 解碼 / PID）
- 執行完成後回到 spinloop 輪詢等待
- 不阻塞、不呼叫 delay()
- PIO 應用：
  - 硬體 PWM：精確頻率 / duty cycle，不受中斷抖動影響
  - Encoder 正交解碼：硬體計數，零 CPU 開銷
- IRQ 明確綁定至 Core 1
- 所有緩衝區靜態分配，禁止 new/malloc
- 關鍵熱路徑函數加 `__not_in_flash_func()` 置入 SRAM

## 核心間通訊

無鎖雙緩衝區（跨 Core 傳遞命令的唯一方式）：

```cpp
static struct SlaveCommand cmdBuf[2];
static std::atomic<uint8_t> cmdWriteIdx{0};
```

**⚠️ 關鍵：volatile 不具備記憶體屏障功能** — `volatile` 只阻止編譯器優化變數，無法保證雙核心間的資料可見性。RP2350 的 SCU（Snoop Control Unit）雖保證 cache coherency，但執行序重排仍需 acquire/release 語意來約束。

**執行序約定：**
- **Core 0（寫入者）**：
  1. 將命令寫入 `cmdBuf[writeIdx]`（普通寫入）
  2. `cmdWriteIdx.store(idx, std::memory_order_release)` — **release 語意**保證以上所有寫入在 store 之前對其他核心可見
- **Core 1（執行者）**：
  1. `uint8_t idx = cmdWriteIdx.load(std::memory_order_acquire)` — **acquire 語意**保證以下所有讀取在 load 之後才執行
  2. `memcpy` 快照 `cmdBuf[writeIdx ^ 1]` 到本地變數

     ~~如果使用 `memory_order_relaxed`，Core 1 可能在 `cmdBuf` 內容還未完全寫入時就開始 memcpy，讀到髒資料。~~
- E-Stop 等緊急旗標用 `std::atomic<bool>`，Core 0 設 `estopActive.store(true, std::memory_order_release)`，Core 1 每次喚醒時 `load(std::memory_order_acquire)` 檢查

## 通訊協議

### COBS 封幀

- 負載最大 254 bytes，編碼後補 `0x00` 終止符
- 接收端以 `0x00` 切割解碼

### Binary Struct

- `__attribute__((packed))`，使用明確寬度型別（int8_t、uint16_t 等）
- Header 格式：`sync (0xA5), msgType, frameSeq, targetID, payloadLen, crc16`
- `targetID`：`0=Hub, 1=Move, 2=Shoot, 3=Storage, 0xFF=Broadcast`
- 每個 struct 後 `static_assert(sizeof(Struct) == EXPECTED_SIZE)`
- Little-endian 字節序（RP2350 原生）

### CRC16-CCITT

- 多項式 `0x1021`，計算範圍不含 crc16 欄位自身
- COBS 解碼後立即 CRC 驗證，失敗靜默丟棄

### 幀序號抗干擾

每個發送幀附帶遞增 `frameSeq`（uint8_t，繞回 0）：
- 接收端記錄最後有效幀的 `frameSeq`
- 收到重複序號視為重送或錯誤，忽略
- 序號跳號（gap）視為掉幀，可觸發安全機制

### 通訊介面一覽

| 鏈路 | 節點 | 介面類型 | 鮑率/速率 | COBS+CRC |
|------|------|----------|-----------|----------|
| Jetson ↔ Hub | GP0/GP1 | USB CDC ACM (Serial) | 921600 bps | ✅ |
| Hub ↔ Move | PIO SM0 | PIO UART | 921600 bps | ✅ |
| Hub ↔ Shoot | PIO SM1 | PIO UART | 921600 bps | ✅ |
| Hub ↔ Storage | PIO SM2 | PIO UART | 921600 bps | ✅ |

### 指令超時保護

- 每顆 Slave 最後有效指令超過 **100ms** 無新幀 → E-Stop 觸發，Core 1 將 PWM 歸零

### 發送規則（無優先級隊列）

921600 bps 下 20 bytes 幀只需 ~200μs，無需軟體排隊：

- **心跳**：Core 0 由 Hardware Timer 每 30ms 觸發發送，直接寫入 TX FIFO
- **遙測**：發送前檢查 `availableForWrite()`，空間不足時 **直接丟棄該幀**，不排隊、不阻塞
- 不需要 Priority Queue — 心跳固定週期發送自然優先，遙測丟了就丟，不會造成無限飢餓

## PIO 開發規則

- PIO 指令延遲 `[n]` 不得超過 31 cycles
- PIO 僅處理 **微秒 (μs) 級**高頻調變與解碼（如 UART bit-banging、Encoder 解碼）
- **毫秒 (ms) 級**延遲與定時觸發一律使用 RP2350 Hardware Timer 或 Core 1 ISR
  — PIO instruction memory 僅 32/64 words，用巢狀迴圈計時 ms 會瞬間吃光空間
- 非 32-bit 倍數資料：手動 push block，不用 autopush
- 釋放匯流排後加 `wait 1 pin` 或足量 nop 防誤觸發
- PIO UART：baud rate = sysclk / (4 × (CLKDIV_INT + CLKDIV_FRAC / 256))
- 每段 PIO 程式附上等價的週期數註解
- PIO 資源精算：2 PIO blocks × 4 SM = 8 SM，Hub 最多用 3 組給 Slave + 預留

## 程式碼風格

- 禁止 C-style cast，強制 `static_cast`、`reinterpret_cast`
- 變數 camelCase，類別 PascalCase，巨集/腳位 UPPER_SNAKE_CASE
- 所有 GPIO 標註實體 pin 編號（例如 `GP16 = Pin 21`）

## E-Stop 安全機制

- 觸發條件：指令逾時 >100ms、PID 輸出 NaN 或超出物理範圍、frameSeq 異常
- Hub 可發送 E-Stop 廣播幀（`targetID=0xFF`）給所有 Slave
- Slave Core 0 收到 E-Stop → 設 `estopActive.store(true, release)` → Core 1 喚醒後歸零 PWM
- Core 1 繼續等待下一個通知，不 reboot
- E-Stop 解除後強制 Slew Rate Limiter 漸進加速

## EMI 實戰注意事項

- UART 通訊線使用雙絞線或隔離線，與馬達電源線保持距離
- 每顆 Pico 電源端加 100μF + 0.1μF 去耦電容
- PIO UART 的 bit-banging 本質比硬體 UART 更能對抗 clock jitter，但仍建議：
  - 近距離（<30cm）直接 PIO UART
  - 中距離加 74LVC1G07 緩衝器
  - 長距離或靠近馬達：**RS-485 收發器** 或 **光耦合隔離**
- 軟體端：frameSeq 抵抗重送/掉幀，CRC 抵抗 bit error，E-Stop 抵抗失聯
- 掉幀率偏高時降低鮑率作為抗干擾手段
