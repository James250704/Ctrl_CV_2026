# Ctrl_CV_2026

棒球機器人 — RP2350 雙核多節點控制韌體（Waveshare RP2350 Plus / Pico2）。

使用 PlatformIO + [Arduino-Pico](https://github.com/earlephilhower/arduino-pico)（RP2350），採 Hub ↔ Move / Shoot / Storage 三個 Slave 節點架構，全車以 COBS + Binary struct + CRC16 通訊。

本 README 記錄**各節點的腳位（GPIO）設定**與**遙控器通道對應參數**。完整開發規範見 [CLAUDE.md](CLAUDE.md)。

---

## 系統拓撲

```
Jetson（USB CDC）⇄ Hub（RP2350）⇄ Move / Shoot / Storage（PIO UART，各一組獨立 SM）
                       ⇄ CRSF 遙控接收機（UART0，GP1 RX）
Ultrasonic（獨立 Pico）— 透過 USB Serial 直接連接 Jetson
```

- **Hub ↔ Slaves**：PIO UART 全雙工（每條 Slave 一組獨立 PIO SM），921600 bps
- **Slave 通訊**：三顆 Slave 共用硬體 UART0（TX=GP0, RX=GP1）收 Hub 控制幀 + 回傳遙測
- **Remote（CRSF）**：Hub Core 1 專職解包，Jetson 斷線（>500ms）時作為控制備援

---

## Hub 腳位設定（RP2350 中央轉發節點）

Hub Core 0 同時負責 USB CDC 收發、PIO UART 對所有 Slave 的 TX/RX；Core 1 專職 CRSF 接收機。

| 功能 | 介面 | 腳位 | 交叉連接 |
|------|------|------|----------|
| Jetson 通訊 | USB CDC (Serial) | — | — |
| CRSF 遙控接收機 | UART0 RX | GP1 | ELRS 接收機 TX → GP1 |
| Move Slave 通訊 | PIO0 SM0 | TX=GP2, RX=GP3 | HUB TX(GP2) -> MOVE RX(GP1); HUB RX(GP3) -> MOVE TX(GP0) |
| Shoot Slave 通訊 | PIO0 SM1 | TX=GP4, RX=GP5 | HUB TX(GP4) -> SHOOT RX(GP1); HUB RX(GP5) -> SHOOT TX(GP0) |
| Storage Slave 通訊 | PIO0 SM2 | TX=GP6, RX=GP7 | HUB TX(GP6) -> STORAGE RX(GP1); HUB RX(GP7) -> STORAGE TX(GP0) |

> **注意**：CRSF @ 420000 baud。GP4/GP5 在 UART1（除錯）與 PIO SM1（Shoot 通訊）間衝突，實作上 PIO SM1 優先，請勿同時啟用 UART1 除錯。

---

## Move Slave 腳位設定（四輪麥克納姆輪）

四輪實測驗證腳位（依 `src/move/main.cpp` `Pin` namespace）：

| 方向 | 馬達 PWM | 馬達 DIR | 編碼器 A | 編碼器 B |
|------|----------|----------|----------|----------|
| **FR**（前右） | GP6 | GP7 | GP14 | GP15 |
| **FL**（前左） | GP9 | GP8 | GP16 | GP17 |
| **BR**（後右） | GP10 | GP11 | GP20 | GP21 |
| **BL**（後左） | GP12 | GP13 | GP18 | GP19 |

### 其他腳位 / 參數

- **通訊 UART0**：TX=GP0, RX=GP1（SlaveComms 內部處理，與 Hub 通訊）
- **狀態 LED**：板載 `LED_BUILTIN`（E-Stop→常亮, 已連線且<100ms 有效命令→閃爍, 斷線→熄滅）
- **PIO 資源**：PIO0 全部 4 SM 做四輪正交編碼器解碼；PIO1 未使用

| 參數 | 值 |
|------|-----|
| 減速比 | 1:27 |
| 實測線數 `ENC_LINE_PPR` | 443 |
| `ENC_COUNTS_PER_REV` | 443 × 4 = 1772 |
| 車身半長/半寬/輪半徑 (m) | 0.25 / 0.25 / 0.05 |
| 馬達極性 `kWheelPolarity[]` | `{1,-1,1,-1}`（FR/BR 反相） |
| 編碼器讀取極性 `kEncReadPolarity[]` | `{1,-1,1,-1}`（FR/BR 鏡像） |
| Strafe 軸方向 | `kAxisStrafePolarity = +1` |
| Forward 軸方向 | `kAxisForwardPolarity = -1` |
| 三軸 S 曲線加速 / 減速 | 4.0 / 12.0 RPS/s |
| 三軸 Jerk | 60.0 RPS/s² |
| PID 出力上限 `PID_MAX_DUTY` | 100% |
| 積分風扇餘裕 `WINDUP_MARGIN_DUTY` | 6% |
| PWM 頻率 | ~20 kHz（clkdiv=6.25, wrap=1000） |

---

## Shoot Slave 腳位設定（上射/下射 BLDC + 俯仰/橫向步進）

依 `src/shoot/main.cpp` `Pin` namespace：

| 功能 | 腳位 |
|------|------|
| 通訊 UART0 TX / RX | GP0 / GP1 |
| UP 上射 BLDC（PWM） | GP2 |
| DOWN 下射 BLDC（PWM） | GP3 |
| FIXED 固定轉速 BLDC（PWM） | GP4 |
| PITCH 俯仰 步進 STEP | GP5 |
| PITCH 俯仰 步進 DIR | GP6 |
| ROW 橫向 步進 STEP | GP7 |
| ROW 橫向 步進 DIR | GP8 |
| STEPPER_ENABLE（低電位致能） | GP9 |

- 無刷馬達為 50Hz servo-style PPM：**1500μs = 中立/停止，2000μs = 全速**
- FIXED 固定轉速無刷脈衝 `PULSE_FIXED` = 1750μs
- 步進馬達：1.8°/step，全步進

---

## Storage Slave 腳位設定（PCA9685 伺服 + Trigger 步進馬達）

依 `src/storage/main.cpp` `Pin` namespace：

| 功能 | 介面 / 腳位 |
|------|-------------|
| 通訊 UART0 TX / RX | GP0 / GP1 |
| PCA9685 I2C1 SDA | GP2 |
| PCA9685 I2C1 SCL | GP3 |
| Trigger 步進 STEP | GP4 |
| Trigger 步進 DIR | GP5 |
| STEPPER_ENABLE（低電位致能） | GP6 |

- **PCA9685**：I2C1（SDA=GP2, SCL=GP3），預設位址 `0x40`；**通道 0~9 連接 10 顆伺服馬達**
- Trigger 步進馬達驅動：A4988 / DRV8825，預設正轉方向
- 伺服脈衝範圍：500~2500μs（每通道可個別設定關閉/開啟位置）

---

## 遙控器通道對應參數（ELRS CRSF）

Hub Core 1 以 420000 baud 接收遙控訊號，Jetson 斷線（>500ms）時作為控制備援。通道值域 **-100 ~ 100**，`rcToInt8()` 套用死區（-10~10 → 0）。

| 遙控通道 | `rc_channels[]` | 參數 | 對應動作 | 目標 Slave |
|----------|-----------------|------|----------|-----------|
| CH1 (Roll) | `[0]` | `vy` | 左右平移 | Move |
| CH2 (Pitch) | `[1]` | `vx` | 前後移動 | Move |
| CH3 (Throttle) | `[2]` | `upSpeed` | 上射速度 | Shoot |
| CH4 (Yaw) | `[3]` | `omega` | 旋轉 | Move |
| CH5 | `[4]` | `downSpeed` | 下射速度 | Shoot |
| CH6 | `[5]` | — | 未使用 | — |
| CH7 | `[6]` | `pitch` | 俯仰（±100 → ±90°） | Shoot |
| CH8 | `[7]` | `row` | 橫向旋轉（±100 → ±90°） | Shoot |
| CH9 | `[8]` | `motorChannel` | PCA9685 通道 0~15 | Storage |
| CH10 | `[9]` | `trigger` | 觸發開關（> 50 視為 1） | Storage |
