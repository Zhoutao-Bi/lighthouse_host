# Design: Dual-Output Pose Stream (BLE + UART)

**Date:** 2026-07-15
**Branch:** `lighthouse_20260714`
**Scope:** Add a hardware UART (UART0) text stream of robot pose alongside the existing BLE broadcaster. No BLE-side changes.

## 1. Background and Goals

The lighthouse tag currently broadcasts pose at ~50 Hz over BLE as 17-byte
manufacturer data. Real-world bring-up and integration with external tooling
(offline loggers, ROS bridges, hand-written parsers, host apps in early
prototyping) benefit from a low-latency, deterministic, **non-Bluetooth** path.
A wired UART line satisfies all of:

- Zero coupling with the BLE stack (works even when the host's BT stack is
  flaky, when nothing is paired, or when we want to ship logs over a tether).
- Cheap to read on the host side: `screen` / `minicom` / a few lines of Python.
- Same data the BLE path emits — single source of truth in `pose_q`.

A clarifying sample the user provided: `x = 0.29 m, y = 0.28 m` — that is the
ASCII payload format the UART must emit.

## 2. Non-Goals

- No new BLE protocol changes. BLE payload layout, advertising cadence, scan
  callback behavior, and module layout under `src/bt_advertise/*` are frozen.
- No console / `printk` coexistence on UART0. Console is moved off; if
  debugging logs are needed, RTT or a temporary overlay is the path.
- No `LIGHTHOUSE_MODE_3D` completion. This is a transport change only.
- No new test framework. Repo has no test infra today (verified by exploration).
- No new transport (NUS / USB CDC / RTT log backend) — only hardware UART0.

## 3. High-Level Architecture

```
   ppi_gpiote_isr  ──►  pos_process_light_signal  ──►  robot_pose_update
                                                            │
                              ┌─────────────────────────────┤
                              ▼                             ▼
                         pose_q_push                    (existing consumer:
                              │                          pose_provider for BLE)
                              │
              ┌───────────────┴───────────────┐
              ▼                               ▼
      bt_advertise                        uart_out  (NEW)
      (existing path, unchanged)               │
                                              ▼
                                  10 ms k_work_delayable
                                  formats line → uart_poll_out → UART0
```

**Key invariants:**

- `pose_q` stays single-producer / single-consumer. `uart_out` does **not**
  pop from `pose_q`; instead it holds its own 1-slot latest-pose snapshot fed
  by `main.c`. This decouples the two transports entirely so that a stalled
  UART cannot drop BLE packets and vice versa.
- The 10 ms main-loop tick is the single heartbeat for pose push. Both
  consumers receive at most one fresh packet per tick.
- BLE advertising and the UART timer are independent; failure of either does
  not affect the other.

## 4. Module Design: `src/uart_out/`

Three new files, mirroring the layout of `src/bt_advertise/`:

### `src/uart_out/uart_out.h`

```c
#pragma once
#include <zephyr/kernel.h>
#include "lighthouse_pkt.h"   /* shared struct, see §4.3 below */

int  uart_out_init(const struct device *uart_dev);
void uart_out_push(const lighthouse_pkt_t *pkt);  /* copies pkt */
```

`lighthouse_pkt_t` is currently `main.c`'s private struct. Two options:

- **(A — chosen)** Move `lighthouse_pkt_t` definition into a small new header
  `src/lighthouse_pkt.h` and have `main.c` include it. Cleanest cross-module
  sharing.
- (B) Keep it in `main.c`, have `uart_out` define its own minimal pose struct.

We choose (A): it is a 7-field struct already used identically in both BLE
serialization and the new UART path; duplicating it invites drift.

### `src/uart_out/uart_out.c`

```c
#define UART_OUT_PERIOD_MS  10
#define UART_OUT_LINE_MAX   64   /* "x=0.000 y=0.000 z=0.000 id=0x1234 mode=2D\r\n" */

static const struct device *uart_dev;
static lighthouse_pkt_t latest;
static struct k_work_delayable uart_work;
static char line[UART_OUT_LINE_MAX];

static void uart_out_work_handler(struct k_work *work);
static int  uart_out_send_line(void);

int uart_out_init(const struct device *dev)
{
    if (!device_is_ready(dev)) return -ENODEV;
    uart_dev = dev;

    /* TX-only: nothing else to configure (board DTS sets 115200 8N1). */
    k_work_init_delayable(&uart_work, uart_out_work_handler);
    k_work_schedule(&uart_work, K_MSEC(UART_OUT_PERIOD_MS));
    return 0;
}

void uart_out_push(const lighthouse_pkt_t *pkt)
{
    /* drop-oldest = overwrite; matches pose_q semantics */
    latest = *pkt;
}

static void uart_out_work_handler(struct k_work *work)
{
    if (uart_dev) uart_out_send_line();
    k_work_schedule(&uart_work, K_MSEC(UART_OUT_PERIOD_MS));
}

static int uart_out_send_line(void)
{
    /* Format the line. Width is bounded; safe against %f overflow. */
    int n = snprintf(line, sizeof(line),
                     "x=%.3f y=%.3f z=%.3f id=0x%04x mode=%s\r\n",
                     (double)latest.x_le, (double)latest.y_le,
                     (double)latest.z_le,
                     ((uint16_t)latest.id_hi << 8) | latest.id_lo,
                     (latest.mode == 0x03) ? "3D" : "2D");
    if (n <= 0 || n >= (int)sizeof(line)) return -EINVAL;

    /* Blocking per-byte; at 115200 baud, 32 bytes ≈ 2.8 ms < 10 ms tick. */
    for (int i = 0; i < n; i++) uart_poll_out(uart_dev, (unsigned char)line[i]);
    return 0;
}
```

Rationale for polling API (`uart_poll_out`) over async (`uart_tx`):

- Single direction, line-oriented, fixed small payload.
- No DMA needed; no callback state machine; no risk of dropping a `UART_TX_DONE`
  callback under main-loop contention.
- Worst-case blocking (one byte at 115200) ≈ 87 µs. Whole line ≈ 2.8 ms, well
  inside the 10 ms tick budget. If TX FIFO fills (no listener), `uart_poll_out`
  still completes per byte at hardware pace — not unbounded.

### `src/lighthouse_pkt.h` (extracted)

```c
#pragma once
#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t mode;
    uint8_t id_lo;
    uint8_t id_hi;
    float   x_le;
    float   y_le;
    float   z_le;
} lighthouse_pkt_t;
```

`main.c` then drops its private definition and includes this header.

## 5. Devicetree Overlay Changes

`nrf52833_lighthouse_tag_202607/boards/nrf52833dk_nrf52833.overlay`:

```dts
/* Existing nrf52833dk_nrf52833.overlay keeps leds and ts4231_n1/n2/n3 unchanged. */
/* Do NOT add a chosen { zephyr,console = &uart0; } block.                  */
/* The board DTS already enables uart0 with default pinctrl for the DK      */
/* virtual COM (P0.06 TX / P0.08 RX). No &uart0 {} override is required     */
/* for default pin pair. If a different pin pair is later required, add:     */
/*                                                                         */
/*   &pinctrl {                                                             */
/*     uart0_default_alt: uart0_default_alt {                               */
/*       group1 { psels = <NRF_PSEL(UART_TX, 0, N1)>,                      */
/*                       <NRF_PSEL(UART_RX, 0, N2)>; };                    */
/*     };                                                                   */
/*   };                                                                     */
/*   &uart0 { pinctrl-0 = <&uart0_default_alt>; };                          */
```

Pins default to the DK virtual COM pair (`P0.06` TX, `P0.08` RX). The user
will confirm / change at bring-up if a different pair is wired.

## 6. Kconfig Changes (`prj.conf`)

```ini
# (existing)
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_OBSERVER=y
CONFIG_BT_DEVICE_NAME="lighthouse_robot"
CONFIG_BT_CTLR_ADV_DATA_LEN_MAX=64
CONFIG_BT_CTLR_SCAN_DATA_LEN_MAX=64
CONFIG_BT_PRIVACY=n
CONFIG_BT_SETTINGS=n
CONFIG_ZERO_LATENCY_IRQS=y

# NEW:
CONFIG_SERIAL=y
# Console deliberately disabled — UART0 is dedicated to pose stream.
CONFIG_CONSOLE=n
CONFIG_PRINTK=n
CONFIG_UART_CONSOLE=n
# Pull in the UARTE driver for nRF52833.
CONFIG_UART_NRF_UARTE=y
```

Rationale: `CONFIG_LOG=y` stays for in-driver `LOG_WRN` paths; with `PRINTK=n`
and `CONSOLE=n` and no `CONFIG_LOG_BACKEND_*` enabled, those logs go nowhere
by default — fine for now. If runtime logging is needed, recommend adding
`CONFIG_LOG_BACKEND_RTT=y` later (RTT does not touch UART0).

## 7. `main.c` Integration

Three small edits to `src/main.c`:

1. `#include "uart_out.h"` and `#include "lighthouse_pkt.h"`.
2. Remove the local `typedef struct ... lighthouse_pkt_t;` block (moved to
   `lighthouse_pkt.h`).
3. After `robot_pose_init()` and before `bt_enable`:

```c
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(uart_out_module, LOG_LEVEL_INF);

const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
if (uart_out_init(uart0) != 0) {
    LOG_WRN("uart0 not ready; pose stream disabled");
    /* do not abort — BLE path still works */
}
```

`printk` is intentionally not used here: with `CONFIG_PRINTK=n` /
`CONFIG_CONSOLE=n` the call would be a silent no-op. `LOG_WRN` flows through
the in-tree log backend (currently no backend is enabled — see §6 note);
if `CONFIG_LOG_BACKEND_RTT=y` is added later the same line becomes visible
on the host without touching this code.

4. In the main loop, after `pose_q_push(&pkt)`, add:

```c
uart_out_push(&pkt);
```

5. Remove the existing `printk("x=%.3f y=%.3f ...\n", ...)` — its job is now
   performed by the UART stream.

## 8. Build Wiring (`CMakeLists.txt`)

Add to `target_sources`:

```cmake
src/uart_out/uart_out.c
```

Add to `target_include_directories`:

```cmake
src/uart_out
```

## 9. Error Handling Summary

| Failure | Detection | Action |
|---|---|---|
| `uart0` device not ready | `device_is_ready()` returns false | Log via `printk` (still works pre-CONSOLE-off), set `LED_MODE_ERROR`, BLE continues |
| `snprintf` overflow / format error | return < 0 or ≥ buffer | skip this tick |
| `uart_poll_out` per-byte failure | driver returns error | skip remainder of line; next tick retries with fresh data |
| `k_work_schedule` never called | N/A | not possible — re-schedule at end of handler |
| BLE failure | unrelated to UART | unchanged behavior |

## 10. Testing Strategy

No automated test framework exists in this repo. Verification is manual:

1. **Build:** `west build -b nrf52833dk/nrf52833 nrf52833_lighthouse_tag_202607/`
   should succeed.
2. **Console:** Confirm boot produces no `printk` chatter on UART0.
3. **UART output:** Connect DK virtual COM at 115200 8N1. Expected line every
   10 ms: `x=0.290 y=0.280 z=0.000 id=0xNNNN mode=2D\r\n` (example). Sample
   10 lines and confirm cadence.
4. **BLE unchanged:** With `nRF Connect`, scan, locate device, observe 17-byte
   manufacturer data still being broadcast at ~50 Hz.
5. **Failure isolation:** Power down BT stack (or stop advertising via a
   toggle added later); UART stream must continue unaffected.

## 11. Documentation Updates

`nrf52833_lighthouse_tag_202607/README.md` — append a "Dual Output (BLE + UART)"
section:

- Pinout for UART0 on DK.
- Baud / format expected by the host.
- Note that console/`printk` has been moved off UART0; for live logs use RTT.
- Reference to `lighthouse_pkt.h` for the shared packet struct.

## 12. Risks and Open Items

- **`ppi.c` build break** flagged during exploration: `ppi_gpiote_isr` is
  declared via `ISR_DIRECT_DECLARE` but no corresponding definition is
  visible. The repo does not build today (`build/` has no `.elf`). This is a
  pre-existing bug, not caused by this change. If discovered during `west
  build`, raise as a follow-up issue; do not bundle the fix into this spec.
- **Default pin assumption** (P0.06 / P0.08) — must be confirmed against the
  user's hardware wiring before flashing.
- **Working tree is dirty** with two unstaged changes (`README.md` and
  `bt_advertise.c` rate bump). They are unrelated to this spec and should be
  stashed / committed separately.