# 系統架構圖

```mermaid
graph LR
    subgraph 上位機["上位機 (Jetson Orin Nano)"]
        JETSON_AI["YOLOv26n 影像辨識<br/>AI 自動移動射擊"]
    end

    subgraph 遙控器["遙控器"]
        DBR4["DBR4 控制器"]
    end

    subgraph 中間位["中間位 (Pico 2 Hub)"]
        HUB["Pico 2 Hub"]
    end

    subgraph 下位機1["下位機 1: 移動平台 (Pico 2)"]
        P1["Pico 2 MCU"]
        P1_DEV["▲ 麥克那姆輪 ×4<br/>▲ BMI270 IMU<br/>▲ 超音波測距 ×6"]
    end

    subgraph 下位機2["下位機 2: 儲球機構 (Pico 2)"]
        P2["Pico 2 MCU"]
        P2_DEV["▲ 伺服馬達 ×16<br/>▲ 步進馬達 ×1 (向上輸送)<br/>▲ 直流減速馬達 ×2"]
    end

    subgraph 下位機3["下位機 3: 射球機構 (Pico 2)"]
        P3["Pico 2 MCU"]
        P3_DEV["▲ 無刷馬達 ×3<br/>▲ 步進馬達 ×2 (水平/仰角)"]
    end

    %% 下行
    DBR4 -->|UART 命令| HUB
    JETSON_AI -->|USB UART 命令| HUB
    HUB -->|PIO UART / COBS| P1
    HUB -->|PIO UART / COBS| P2
    HUB -->|PIO UART / COBS| P3

    P1 --- P1_DEV
    P2 --- P2_DEV
    P3 --- P3_DEV

    %% 上行
    P1 -.->|輪速 / IMU / 距離| HUB
    P2 -.->|伺服角度 / 球數| HUB
    P3 -.->|馬達 RPM / 角度| HUB
    HUB -.->|彙總狀態| JETSON_AI

    classDef jetson fill:#1a73e8,color:#fff,stroke:#0d47a1,stroke-width:2px
    classDef remote fill:#7b1fa2,color:#fff,stroke:#4a148c,stroke-width:2px
    classDef hub fill:#e8710a,color:#fff,stroke:#bf5e03,stroke-width:2px
    classDef pico fill:#f5f5f5,color:#000,stroke:#666,stroke-width:2px
    classDef device fill:#e8f5e9,color:#000,stroke:#388e3c,stroke-width:1px,dashed

    class JETSON_AI jetson
    class DBR4 remote
    class HUB hub
    class P1,P2,P3 pico
    class P1_DEV,P2_DEV,P3_DEV device
```

## Hub 內部架構

```mermaid
graph TB
    subgraph 硬體介面層["硬體介面層"]
        USB_IF["USB CDC (from Jetson)"]
        UART_IF["UART (from DBR4)"]
    end

    subgraph 收發層["收發層"]
        RX["UART RX"]
        TX["UART TX"]
    end

    subgraph 編解碼層["編解碼層"]
        COBS["COBS 解碼／編碼"]
        STRUCT["Binary Struct<br/>Serialize / Deserialize"]
    end

    subgraph 交換層["交換層"]
        ROUTE["封包路由判斷"]
        DOWNLINK["下行分派"]
        UPLINK["上行彙整"]
    end

    subgraph 下位機介面["下位機介面"]
        PIO_IF["PIO 模擬 UART ×3<br/>(Move / Storage / Shoot)"]
    end

    %% 下行路徑
    USB_IF --> RX
    UART_IF --> RX
    RX --> COBS
    COBS --> STRUCT
    STRUCT --> ROUTE
    ROUTE --> DOWNLINK
    DOWNLINK --> PIO_IF

    %% 上行路徑
    PIO_IF --> UPLINK
    UPLINK --> ROUTE
    ROUTE --> STRUCT
    STRUCT --> COBS
    COBS --> TX
    TX --> USB_IF
    TX --> UART_IF

    classDef hw fill:#ffe0b2,color:#000,stroke:#e65100
    classDef io fill:#fff3e0,color:#000,stroke:#f57c00
    classDef codec fill:#fff8e1,color:#000,stroke:#f9a825
    classDef route fill:#fce4ec,color:#000,stroke:#c62828
    classDef iface fill:#e3f2fd,color:#000,stroke:#1565c0

    class USB_IF,UART_IF hw
    class RX,TX io
    class COBS,STRUCT codec
    class ROUTE,DOWNLINK,UPLINK route
    class PIO_IF iface
```

## 資料流向

| 方向 | 路徑 | 內容 |
|------|------|--------|
| 下行 | Jetson → Hub → 下位機 | 移動指令、儲球指令、射球參數 |
| 下行 | DBR4 → Hub → 下位機 | 遙控器操作命令 |
| 上行 | 下位機 1 → Hub → Jetson | 四輪轉速、六軸超音波距離、IMU 姿態 |
| 上行 | 下位機 2 → Hub → Jetson | 16 顆伺服角度、步進馬達位置、直流馬達轉速、儲球數量 |
| 上行 | 下位機 3 → Hub → Jetson | 三顆無刷馬達 RPM、水平/仰角步進角度、射擊狀態 |
