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

**当前固件源码支持 3 个 TS4231 sensor**（devicetree overlay 声明 N1/N2/N3 三节点），
但 nrf52833dk 开发板上**只有 N1 实际布线**（E=P1.09, D=P0.11）；N2/N3 节点存在但硬件
未接。要扩到 3 sensor 见 `限制 §1`。

## 已实现的功能

### 1. BLE 广播（替代原 raw radio）

- 设备名 `lighthouse_robot`
- **non-connectable** undirected advertising
- **17 字节** manufacturer data（2 字节 Company ID + 15 字节 payload）
- 广播数据更新周期 **20 ms**（`BT_ADV_REFRESH_PERIOD_MS`），主循环每 10 ms 把最新位姿压入 4 项环形队列 `pose_q`，BLE 侧按 drop-oldest 取最新
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

- `pose_provider()` 回调每 20 ms 被 `bt_advertise` 调一次（取 `pose_q` 最新项）
- 输出格式见下节
- 接收端（手机 nRF Connect / 其他 nrf52833）按格式解就能拿 `robot_pose_t`

## BLE Payload 协议

`BT_DATA_MANUFACTURER_DATA` 段（**17 字节** = 2 字节 Company ID + 15 字节 `lighthouse_pkt_t`）：

| 偏移 | 长度 | 字段 | 类型 |
|---|---|---|---|
| 0 | 2 | Company ID = `0xFFFF, 0xFFFF` | uint16 LE（测试 ID） |
| 2 | 1 | `mode` | uint8 (`0x01`=2D, `0x03`=3D；由 `LIGHTHOUSE_MODE_3D` 决定) |
| 3 | 1 | `id_lo` | uint8 (DEVICEADDR[0] 低字节) |
| 4 | 1 | `id_hi` | uint8 (DEVICEADDR[0] 高字节) |
| 5 | 4 | `position.x` | float LE |
| 9 | 4 | `position.y` | float LE |
| 13 | 4 | `position.z` | float LE |
| 总计 | **17** | | |

> 头部是 `0xFF 0xFF` 而**不是** `0x04 0x00`；这与 nRF Connect 默认 manufacturer 解析器显示的字节序相反，是底层测试 ID。

**所有多字节字段都是 little-endian**（nrf52833 ARM Cortex-M4 native byte order）。

> **注意：当前 17 字节 layout 没有 `valid` / `valid_sensor_count` / `valid_sensor_mask` / `heading_deg` 字段**。
> 这几个字段在 commit `30e978f` (D2) 之后被替换为 `mode` + `id` + 3 个 float 坐标。Qt 端 / 网关
> 解析代码必须按上表实现，不要再用旧 34 字节格式。原 raw radio (`0x24` 头 + 24-bit CRC) 已废弃。

> **field type 注意**：x/y/z 是 **`float` (IEEE 754 single)**，不是 `double`。nrf52833 单精度 FPU 硬件加速，
> 占用 4 字节；按 double 解会读错字节。`mode` 为 `0x01` 时 z 永远 0（2D 模式，参见 `限制 §6`）。

## 2D / 3D 模式

`src/lighthouse_config.h` 提供编译期开关：

```c
#define LIGHTHOUSE_MODE_3D 0   /* 0 = 2D (默认)  1 = 3D (未实现, -ENOTSUP) */
```

- **2D 模式 (默认)**：`pos.c` 用 `lighthouse_get_position_simple` 把 z 固定到 0，单 lighthouse 基站即可工作。
- **3D 模式**：未实现，`pos_get_sensor_position` 在 3D 模式下返回 `-ENOTSUP`；要启用需先：
  1. 在 `pos.c` 实现 `lighthouse_get_position_3d()`（双基站 skew lines 解算）
  2. 把 `LIGHTHOUSE_MODE_3D` 改成 1
  3. 在 `main.c` 提供第二个 lighthouse 的 `calib_data_b`（同样 `lighthouse_calibrate` 一次）

## 文件结构

```
nrf52833_lighthouse_tag_202607/
├── CMakeLists.txt              # 10 源文件
├── prj.conf                    # BT + GPIO + ZERO_LATENCY_IRQS
├── boards/
│   └── nrf52833dk_nrf52833.overlay  # user_led + ts4231_n1/n2/n3 节点
│                                     # （DK 上仅 N1 实际布线；N2/N3 节点存在但未接）
├── dts/bindings/
│   └── lighthouse_robot,ts4231/    # 自定义 binding
│       └── lighthouse_robot,ts4231.yaml
└── src/
    ├── main.c                  # 编排：init + calibrate + BLE + pose_q 主循环
    ├── lighthouse_config.h     # LIGHTHOUSE_MODE_3D 编译期开关
    ├── led/                    # P0.20 状态灯（init/on/off/toggle/set_mode/notify_rx）
    ├── ts4231/                 # TS4231 状态机 + 多 sensor 编排
    │   ├── ts4231.{c,h}
    │   └── ts4231_sensors.{c,h}
    ├── ppi/                    # PPI/GPIOTE/TIMER3 裸寄存器 + ZLI ISR
    │   ├── ppi.{c,h}
    │   └── timer.{c,h}
    ├── pos/                    # P3P + Kalman + 2D/3D 解算
    │   └── pos.{c,h}
    ├── robot_pose/             # 3 sensor → 中心位姿（含 calibrated ref 姿态）
    │   └── robot_pose.{c,h}
    ├── bt_advertise/           # BLE 广播（payload provider + 20 ms 周期更新）
    │   └── bt_advertise.{c,h}
    └── bt_scan/                # BLE 扫描（不连接，仅 RX 触发 LED）
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

1. **单 sensor**：devicetree overlay 声明 N1/N2/N3 三个节点，**但 nrf52833dk 上仅 N1
   (E=P1.09, D=P0.11) 实际布线**；N2/N3 节点存在但未接物理 sensor，`ts4231_init()` 对它们
   会 `ts4231_waitForLight` 超时返回 `-ETIMEDOUT`（受 `限制 §2` 影响会卡住）。
   要扩到 3 sensor 需在硬件上把 N2/N3 接到 `(P0.03/P0.02)` 和 `(P0.09/P0.10)`，并补 `ts4231_n2/3_handle()`
   的初始化逻辑（main.c 已经按 3-handle 数组循环）。
2. **TS4231 故障容错差**：`ts4231_waitForLight()` 内部死循环。没接硬件时 `ts4231_init()`
   会卡住，LED 保持 INIT 闪。源码就是这样，**没改**。
3. **不做 GATT 连接 / Mesh / Pairing**：纯广播。手机 nRF Connect 能看到，标准 BLE 库能
   读 manufacturer data，但**两 robot 板之间不互连**。
4. **校准数据硬编码**：运行时不能再校准。要重新校准需要改 `main.c` 重烧。
5. **lighthouse_host Qt 端协议不兼容**：原 raw radio (`0x24` 头 + 24-bit CRC) 协议已废，
   Qt 端要按上面 17 字节 layout 重写解析。
6. **pose z 永远 0**：`lighthouse_get_position_simple` 把 z 固定到 0（V1 2D 平面
   假设），3D 高度需要换 V2 lighthouse 协议或开启 `LIGHTHOUSE_MODE_3D`（目前 3D
   算法未实现，返回 `-ENOTSUP`，参见 `2D / 3D 模式`）。
7. **aod.c 没移植**：源码里的 RF AoD 是 V2 lighthouse 用的，本仓只 port V1
   光学 + 算法。

## 已知问题 / Known Bugs

以下 bug 在最新代码已修复（参见 commit 之后的本地 edit）：

- ~~**B1** `ppi_init()` 从未被调用~~ — 已修复：`main.c` 现在调用 `ppi_init(TIMER_3)`
  完成 `IRQ_DIRECT_CONNECT(GPIOTE_IRQn, ...)`。原代码 `irq_enable(GPIOTE_IRQn)` 已就绪但
  vector 未连接 → GPIOTE 中断会触发未注册 handler。
- ~~**B2** `pose_q` 在主循环里被立即 drain 干净，BLE provider 永远拿不到包~~ — 已修复：
  `main.c` 主循环只 push 不 drain；`pose_q_push` 实现 drop-oldest；`pose_q_push`/`pose_q_pop`
  调整 write-before-publish / read-before-consume 顺序。
- ~~**B3** `ts4231_init` retry 循环读 stale `current_state`~~ — 已修复：`ts4231.c` 在第一次
  `ts4231_configDevice()` 之后立即 `ts4231_checkBus()` 刷新 `current_state`，再进入 while 判定。

仍存的小问题：

- **B5** Dead code（未清理，不影响功能）：`ppi_gpiote_init`、`ts4231_sensor_start_from_config`、
  `ts4231_sensor_attach_ppi` (无 index 变体)、`main.c::on_pulse` 回调均未被调用。
- **B6** `ts4231_sensor_start_from_config` 忽略 `ts4231_init()` 返回值。
- **B7** `pos.c::lighthouse_calibrate` 用 `((double *)&qa)[row]` 做 type-punning —— 在 nrf52833
  Cortex-M4 上工作但严格意义上是 UB，未来换平台要改成 `union` 或显式 `memcpy`。

## Commit 历史

```
2ffde13 D3: 3 sensors + 17B payload + pose_q drop-oldest + 50ms timeout
30e978f D2: pose serialization into BLE manufacturer data (replaces counter)
7a4abec C:  pos.c (P3P + Kalman) + robot_pose.c (3-sensor fusion)
039595e B:  PPI + TIMER3 capture (raw HAL port, IRQ_DIRECT_CONNECT)
3a307ef A:  TS4231 driver (Zephyr GPIO API port, BSD-preserved)
fb80523 D1: BLE broadcaster + LED status indicator + TS4231 devicetree binding
69bb998 Add nrf52833_lighthouse_tag_202607 subdir as self-contained device firmware
```

6 个 device-firmware commit + 1 个 wrapper commit，全在 `Zhoutao-Bi/lighthouse_host` GitHub 仓的
`lighthouse_20260714` 分支。

> README 之后，本地又修了一组 bug（B1–B4，参见 `已知问题 / Known Bugs`）。
