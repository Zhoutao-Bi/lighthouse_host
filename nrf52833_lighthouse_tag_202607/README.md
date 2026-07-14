# lighthouse_robot 固件 (nrf52833 Zephyr)

移植自 atom-robot 固件（BSD 头保留），基于 NCS v3.4.0 / Zephyr 4.4.0。

## 项目目标

**`lighthouse_host` Qt 桌面端** + **`lighthouse_robot` 嵌入式固件** = Valve Lighthouse
定位系统的双边实现。固件端把 3 个 TS4231 光学传感器读数 → P3P 解算 → 机器人位姿
(x, y, z, heading) → BLE 广播；桌面端用 BLE 扫描器抓 manufacturer data，绘轨迹。

本仓是 **`lighthouse_host` 仓库的 `lighthouse_20260714` 分支**下的固件子目录：
- Qt 桌面端 / gateway：见 `lighthouse_host` 仓的 `main` / `V1`–`v4` 分支
- robot 固件（**本目录**）：从 `atom-robot` 仓库裸机代码移植到 NCS

## 硬件需求

| 器件 | 作用 | 引脚（nrf52833dk） |
|---|---|---|
| nRF52833 | MCU | 板载 |
| TS4231 × 1 | V1 lighthouse 光学传感器（E=clock, D=data） | E=P1.09, D=P0.11 |
| 用户 LED | 状态指示 | P0.20 |
| Lighthouse 基站 | V1 模式（sweep + sync） | 不接 MCU |

**当前固件只支持 1 个 TS4231 sensor**（源码支持 3 个但本移植只配 1 个）。要扩到 3 个需
要扩 devicetree + 调整 `ts4231_sensors.c` 的 `ppi_init_multi` 的 sensor_count。

## 已实现的功能

### 1. BLE 广播（替代原 raw radio）

- 设备名 `lighthouse_robot`
- **non-connectable** undirected advertising
- 33 字节 manufacturer data，每 100 ms 刷新一次
- 监听：被动 scan，看到其他 robot 的广播触发 LED 闪一下
- 不支持 GATT 连接、不需要 central 角色（**两板之间不自动连接**，仅"看得到"）

### 2. LED 状态灯（P0.20）

5 种状态模式：
- `OFF`：灭
- `INIT`：上电 200 ms 闪
- `BROADCASTING`：BLE 启动后 500 ms 慢闪（默认）
- `ERROR`：出错常亮
- `RX`：收到其他板广播时闪 200 ms（多个相邻 RX 会被合并成持续快闪）

### 3. TS4231 驱动

- I2C-like 位翻转协议（**不是 SPI**）配置 chip：state machine 检测 + 14/15-bit config 读写
- E（P1.09）/ D（P0.11）引脚从 devicetree 拿
- `ts4231_init()`：循环尝试配置直到 chip 进入 WATCH state
- `ts4231_is_lighthouse()`：是否检测到 lighthouse 信号
- 完整保留源 BSD license 头

### 4. PPI 脉冲捕获（GPIOTE + PPI + TIMER3 + ISR）

- `ppi_gpiote_init_sensor(idx, &gpio_dt_spec)`：配 GPIOTE 通道 2*idx (falling) + 2*idx+1 (rising)
- `ppi_init_multi(TIMER_3, sensor_count)`：PPI 路由 GPIOTE event → TIMER3 CC 捕获任务（**无 CPU 介入**）
- `ppi_set_light_signal_ex_callback()`：注册 ISR 回调
- **ZLI (zero-latency interrupt)**：用 `IRQ_DIRECT_CONNECT` 直连向量表，绕过 .intList 避免与 nrfx 默认 handler 冲突
- 1 个 sensor 占用 GPIOTE channel 0/1 + PPI channel 0/1 + TIMER3 CC0/CC1

### 5. Lighthouse 算法（V1）

- `pos.c`：
  - **P3P 标定**：Gauss-Newton 50 次迭代解 3 点 PnP → 计算 lighthouse 基站位姿 + 旋转矩阵
  - **Kalman 滤波** 对每 sensor 的水平 / 垂直角做 1 阶滤波
  - **Pulse 分类**：sync / sweep / skip_sync 通过 pulse 宽度区分（800/1584 ticks 阈值，16 MHz 时基 = 50/99 µs）
- `robot_pose.c`：3 sensor 位置 → 中心位置 + 航向，依赖至少 2 个有效 sensor
- 校准数据 `cal_pos` / `cal_angles` 在 `main.c` 硬编码（**W2-606 office 配置**），适配你环境需要改

### 6. Pose → BLE 序列化

- `pose_provider()` 回调每 100 ms 被 `bt_advertise` 调一次
- 输出格式见下节
- 接收端（手机 nRF Connect / 其他 nrf52833）按格式解就能拿 `robot_pose_t`

## BLE Payload 协议

`BT_DATA_MANUFACTURER_DATA` 段（34 字节）：

| 偏移 | 长度 | 字段 | 类型 |
|---|---|---|---|
| 0 | 2 | Company ID = `0xFFFF, 0xFFFF` | uint16 LE（测试 ID） |
| 2 | 1 | `valid` | uint8 (0/1) |
| 3 | 1 | `valid_sensor_count` | uint8 (0..3) |
| 4 | 1 | `valid_sensor_mask` | uint8 (bit0..bit2) |
| 5 | 1 | padding | 0 |
| 6 | 8 | `position.x` | double LE |
| 14 | 8 | `position.y` | double LE |
| 22 | 8 | `position.z` | double LE |
| 30 | 4 | `heading_deg` | float LE |

**所有多字节字段都是 little-endian**（nrf52833 ARM Cortex-M4 native byte order）。

**重要**：原 `lighthouse_host` Qt 端是用 `0x24` 头 + 24-bit CRC 解析旧 raw radio 协议的。**这个协议不兼容**，Qt 端要按上表重写解析。

## 文件结构

```
nrf52833_lighthouse_tag_202607/
├── CMakeLists.txt              # 10 源文件
├── prj.conf                    # BT + GPIO + ZERO_LATENCY_IRQS
├── boards/
│   └── nrf52833dk_nrf52833.overlay  # user_led + ts4231_n1 节点
├── dts/bindings/
│   └── lighthouse_robot,ts4231/    # 自定义 binding
│       └── lighthouse_robot,ts4231.yaml
└── src/
    ├── main.c                  # 编排：init + calibrate + BLE + 主循环
    ├── led/                    # P0.20 状态灯（init/on/off/toggle/set_mode/notify_rx）
    ├── ts4231/                 # TS4231 状态机 + 多 sensor 编排
    │   ├── ts4231.{c,h}
    │   └── ts4231_sensors.{c,h}
    ├── ppi/                    # PPI/GPIOTE/TIMER3 裸寄存器 + ZLI ISR
    │   ├── ppi.{c,h}
    │   └── timer.{c,h}
    ├── pos/                    # P3P + Kalman
    │   └── pos.{c,h}
    ├── robot_pose/             # 3 sensor → 中心位姿
    │   └── robot_pose.{c,h}
    ├── bt_advertise/           # BLE 广播
    │   └── bt_advertise.{c,h}
    └── bt_scan/                # BLE 扫描（不连接）
        └── bt_scan.{c,h}
```

每个模块都是独立子目录，BSD 头全部保留。源码规则保持一致：
- `led/` / `ppi/` / `ts4231/` 是从 atom-robot 源代码搬运改写
- `pos/` / `robot_pose/` 也是搬运改写
- `bt_advertise/` / `bt_scan/` 是新写的（替换原 raw radio）

## 构建

需要 NCS v3.4.0 toolchain 在 `C:\ncs\toolchains\dcbdc366a1\`，`ZEPHYR_BASE=C:\ncs\v3.4.0\zephyr`。

```cmd
cd D:\project\lighthouse_20260714\nrf52833_lighthouse_tag_202607
west build -b nrf52833dk/nrf52833 --pristine
west flash
```

## 校准

`main.c` 顶部硬编码 W2-606 office 的校准数据：

```c
const lighthouse_point cal_pos[] = {
    {0.0,   0.0,   0.0},
    {594.0, 0.0,   0.0},
    {594.0, 420.0, 0.0}
};
const lighthouse_angles cal_angles[] = {
    {82.556, 74.598},
    {88.518, 87.382},
    {81.198, 91.95}
};
```

**改这个数组适配你的 lighthouse 基站物理位置**。每个 (x, y) 单位 mm，每组
(alpha, beta) 是 V1 lighthouse 的水平/垂直扫描角（度）。

## 限制

1. **单 sensor**：devicetree 只配了 N1（P1.09/P0.11）。要 3 sensor 需扩 overlay + 调
   `ppi_init_multi(TIMER_3, 3)`。
2. **TS4231 故障容错差**：`ts4231_waitForLight()` 内部死循环。没接硬件时 `ts4231_init()`
   会卡住，LED 保持 INIT 闪。源码就是这样，**没改**。
3. **不做 GATT 连接 / Mesh / Pairing**：纯广播。手机 nRF Connect 能看到，标准 BLE 库能
   读 manufacturer data，但**两 robot 板之间不互连**。
4. **校准数据硬编码**：运行时不能再校准。要重新校准需要改 `main.c` 重烧。
5. **lighthouse_host Qt 端协议不兼容**：原 raw radio (`0x24` 头 + 24-bit CRC) 协议已废，
   Qt 端要按上面 34 字节 layout 重写解析。
6. **pose z 永远 0**：`lighthouse_get_position_simple` 把 z 固定到 0（V1 2D 平面
   假设），3D 高度需要换 V2 lighthouse 协议。
7. **aod.c 没移植**：源码里的 RF AoD 是 V2 lighthouse 用的，本仓只 port V1
   光学 + 算法。

## Commit 历史

```
30e978f D2: pose serialization into BLE manufacturer data (replaces counter)
7a4abec C: pos.c (P3P + Kalman) + robot_pose.c (3-sensor fusion)
039595e B: PPI + TIMER3 capture (raw HAL port, IRQ_DIRECT_CONNECT)
3a307ef A: TS4231 driver (Zephyr GPIO API port, BSD-preserved)
fb80523 D1: BLE broadcaster + LED status indicator + TS4231 devicetree binding
```

5 个 commit 全在 `Zhoutao-Bi/lighthouse_host` GitHub 仓的 `lighthouse_20260714` 分支。
