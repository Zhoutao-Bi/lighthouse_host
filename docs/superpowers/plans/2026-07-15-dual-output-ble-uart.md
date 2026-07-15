# Dual-Output Pose Stream (BLE + UART) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a 100 Hz UART0 text pose stream to the lighthouse tag firmware alongside the existing 50 Hz BLE broadcast, by extracting the shared `lighthouse_pkt_t` struct and adding a new `uart_out` transport module.

**Architecture:** New `src/uart_out/` module (mirrors `src/bt_advertise/` layout) consumes a 1-slot latest-pose snapshot fed by `main.c`'s 10 ms tick; formats ASCII line and writes via `uart_poll_out`. BLE path is untouched. Console/`printk` is removed from UART0 so the pose stream owns that line.

**Tech Stack:** Zephyr v3.4.0 (NCS), C11, nRF52833 DK, devicetree overlay, `k_work_delayable`, `uart_poll_out`.

**Repo entry point:** `nrf52833_lighthouse_tag_202607/`

## Global Constraints

- All file paths are relative to the repo root unless absolute.
- The repo has **no** automated test framework (verified in spec §10). Tasks verify via `west build` only; runtime checks are listed in spec §10 for the user to perform on hardware.
- `printk` is disabled after this plan lands (`CONFIG_PRINTK=n`). Do not add new `printk(...)` calls in any task. Use `LOG_WRN/LOG_INF` via `<zephyr/logging/log.h>` instead.
- `lighthouse_pkt_t` MUST be defined exactly once in `src/lighthouse_pkt.h` (Tasks 1) and included wherever used.
- UART device is `DT_NODELABEL(uart0)` on the nRF52833DK. Default pins are P0.06 (TX) / P0.08 (RX).
- Baud rate / framing are fixed by the board DTS at 115200 8N1 — do not override.

---

### Task 1: Extract `lighthouse_pkt_t` into a shared header

**Files:**
- Create: `nrf52833_lighthouse_tag_202607/src/lighthouse_pkt.h`
- Modify: `nrf52833_lighthouse_tag_202607/src/main.c:30-37` (replace private struct with `#include "lighthouse_pkt.h"`)

**Interfaces:**
- Produces: `typedef struct __attribute__((packed)) { uint8_t mode; uint8_t id_lo; uint8_t id_hi; float x_le; float y_le; float z_le; } lighthouse_pkt_t;`

- [ ] **Step 1: Create `src/lighthouse_pkt.h`**

Write exactly:

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

- [ ] **Step 2: Modify `src/main.c` to include the new header and drop the local definition**

In `src/main.c`:
- Delete lines 30-37 (the `typedef struct __attribute__((packed)) { ... } lighthouse_pkt_t;` block).
- Add `#include "lighthouse_pkt.h"` near the top with the other `#include` lines (after the existing `#include "lighthouse_config.h"`).

Result: `lighthouse_pkt_t` continues to be used as before inside `main.c` (struct definitions in `pose_q`, `pose_q_push`, `pose_q_pop`, `pose_provider`), but the type now comes from the new header.

- [ ] **Step 3: Build to confirm no regressions**

Run from repo root:
```bash
west build -b nrf52833dk_nrf52833 -p auto nrf52833_lighthouse_tag_202607/
```
Expected: build succeeds. (Pre-existing `ppi.c` ISR definition gap, flagged in spec §12, is out of scope here — if the build fails *only* on that, document it in the commit message and stop; otherwise fix the regression introduced by this task before committing.)

- [ ] **Step 4: Commit**

```bash
git add nrf52833_lighthouse_tag_202607/src/lighthouse_pkt.h nrf52833_lighthouse_tag_202607/src/main.c
git commit -m "refactor: extract lighthouse_pkt_t into shared header"
```

---

### Task 2: Create `uart_out` module header

**Files:**
- Create: `nrf52833_lighthouse_tag_202607/src/uart_out/uart_out.h`

**Interfaces:**
- Consumes: `lighthouse_pkt_t` (from `lighthouse_pkt.h`).
- Produces:
  - `int  uart_out_init(const struct device *uart_dev);`
  - `void uart_out_push(const lighthouse_pkt_t *pkt);`

- [ ] **Step 1: Create the header file**

Write exactly:

```c
#pragma once
#include <zephyr/kernel.h>
#include "lighthouse_pkt.h"

int  uart_out_init(const struct device *uart_dev);
void uart_out_push(const lighthouse_pkt_t *pkt);
```

- [ ] **Step 2: Commit**

```bash
git add nrf52833_lighthouse_tag_202607/src/uart_out/uart_out.h
git commit -m "feat(uart_out): add public api"
```

---

### Task 3: Implement `uart_out` module

**Files:**
- Create: `nrf52833_lighthouse_tag_202607/src/uart_out/uart_out.c`

**Interfaces:**
- Consumes: `lighthouse_pkt_t`, Zephyr `uart_poll_out`, `k_work_delayable`.

- [ ] **Step 1: Write `src/uart_out/uart_out.c`**

Write exactly:

```c
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <stddef.h>
#include <stdio.h>

#include "uart_out.h"

LOG_MODULE_REGISTER(uart_out, CONFIG_LOG_DEFAULT_LEVEL);

#define UART_OUT_PERIOD_MS 10
#define UART_OUT_LINE_MAX  64

static const struct device *uart_dev;
static lighthouse_pkt_t     latest;
static struct k_work_delayable uart_work;
static char                line_buf[UART_OUT_LINE_MAX];

static void uart_out_send_line(void);

static void uart_out_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (uart_dev == NULL) {
		return;
	}

	uart_out_send_line();
	k_work_schedule(&uart_work, K_MSEC(UART_OUT_PERIOD_MS));
}

static void uart_out_send_line(void)
{
	const char *mode_str = (latest.mode == 0x03) ? "3D" : "2D";
	uint16_t id = (uint16_t)(((uint16_t)latest.id_hi << 8) | latest.id_lo);

	int n = snprintf(line_buf, sizeof(line_buf),
			 "x=%.3f y=%.3f z=%.3f id=0x%04x mode=%s\r\n",
			 (double)latest.x_le,
			 (double)latest.y_le,
			 (double)latest.z_le,
			 id,
			 mode_str);

	if (n <= 0 || n >= (int)sizeof(line_buf)) {
		LOG_WRN("snprintf failed: %d", n);
		return;
	}

	for (int i = 0; i < n; i++) {
		unsigned char c = (unsigned char)line_buf[i];
		uart_poll_out(uart_dev, c);
	}
}

int uart_out_init(const struct device *dev)
{
	if (dev == NULL || !device_is_ready(dev)) {
		return -ENODEV;
	}

	uart_dev = dev;
	k_work_init_delayable(&uart_work, uart_out_work_handler);
	k_work_schedule(&uart_work, K_MSEC(UART_OUT_PERIOD_MS));

	return 0;
}

void uart_out_push(const lighthouse_pkt_t *pkt)
{
	if (pkt == NULL) {
		return;
	}

	latest = *pkt;
}
```

- [ ] **Step 2: Build to confirm compilation**

```bash
west build -b nrf52833dk_nrf52833 -p auto nrf52833_lighthouse_tag_202607/
```
Expected: build fails because `CMakeLists.txt` does not yet reference `uart_out.c`. That is OK for this task — proceed.

- [ ] **Step 3: Commit**

```bash
git add nrf52833_lighthouse_tag_202607/src/uart_out/uart_out.c
git commit -m "feat(uart_out): add 10 ms pose text stream"
```

---

### Task 4: Wire `uart_out` into the build (`CMakeLists.txt`)

**Files:**
- Modify: `nrf52833_lighthouse_tag_202607/CMakeLists.txt:6-27`

- [ ] **Step 1: Update `CMakeLists.txt`**

In `nrf52833_lighthouse_tag_202607/CMakeLists.txt`, modify as follows:

Replace the `target_sources(app PRIVATE ...)` block (lines 6-17) to add `src/uart_out/uart_out.c`:

```cmake
target_sources(app PRIVATE
	src/main.c
	src/led/led.c
	src/bt_advertise/bt_advertise.c
	src/bt_scan/bt_scan.c
	src/ppi/ppi.c
	src/ppi/timer.c
	src/ts4231/ts4231.c
	src/ts4231/ts4231_sensors.c
	src/pos/pos.c
	src/robot_pose/robot_pose.c
	src/uart_out/uart_out.c
)
```

Replace the `target_include_directories(app PRIVATE ...)` block (lines 19-27) to add `src/uart_out`:

```cmake
target_include_directories(app PRIVATE
	src/led
	src/bt_advertise
	src/bt_scan
	src/ppi
	src/ts4231
	src/pos
	src/robot_pose
	src/uart_out
)
```

- [ ] **Step 2: Build to confirm clean compilation**

```bash
west build -b nrf52833dk_nrf52833 -p auto nrf52833_lighthouse_tag_202607/
```
Expected: build still fails only because `prj.conf` and `main.c` are not yet updated (no `CONFIG_SERIAL`, no `uart_out_init` call). That is fine.

- [ ] **Step 3: Commit**

```bash
git add nrf52833_lighthouse_tag_202607/CMakeLists.txt
git commit -m "build: register uart_out module"
```

---

### Task 5: Update `prj.conf` (serial on, console off)

**Files:**
- Modify: `nrf52833_lighthouse_tag_202607/prj.conf`

- [ ] **Step 1: Append the four Kconfig lines at the bottom of `prj.conf`**

After the existing `CONFIG_ZERO_LATENCY_IRQS=y` line, append exactly:

```ini

# --- dual-output pose stream (BLE + UART0) ---
CONFIG_SERIAL=y
CONFIG_CONSOLE=n
CONFIG_PRINTK=n
CONFIG_UART_CONSOLE=n
CONFIG_UART_NRF_UARTE=y
```

- [ ] **Step 2: Build to confirm `CONFIG_SERIAL` is satisfied**

```bash
west build -b nrf52833dk_nrf52833 -p auto nrf52833_lighthouse_tag_202607/
```
Expected: build still fails on missing `uart_out_init` in `main.c` — fine, fixed in next task.

- [ ] **Step 3: Commit**

```bash
git add nrf52833_lighthouse_tag_202607/prj.conf
git commit -m "kconfig: enable uart0 driver, disable console/printk"
```

---

### Task 6: Confirm devicetree overlay needs no change

**Files:**
- Read-only verification of `nrf52833_lighthouse_tag_202607/boards/nrf52833dk_nrf52833.overlay`

- [ ] **Step 1: Verify no edits needed**

Re-read `boards/nrf52833dk_nrf52833.overlay`. It must contain only `leds`, `aliases`, and `ts4231_n1/n2/n3`. No `chosen { zephyr,console = &uart0; }` block, no `&uart0 {}` override.

If the file already matches the above, no action — skip Steps 2-3.

If the file currently binds `zephyr,console = &uart0` (because the user previously ran with console on UART0), delete that line. Do **not** add any `&uart0 {}` block.

- [ ] **Step 2: Build (sanity)**

```bash
west build -b nrf52833dk_nrf52833 -p auto nrf52833_lighthouse_tag_202607/
```
Expected: build still fails on missing `uart_out_init` in `main.c` — fine, fixed in next task.

- [ ] **Step 3: Skip commit if no changes; otherwise commit**

```bash
git add nrf52833_lighthouse_tag_202607/boards/nrf52833dk_nrf52833.overlay
git commit -m "dts: leave uart0 default (no console binding)"
```

---

### Task 7: Integrate `uart_out` into `main.c`

**Files:**
- Modify: `nrf52833_lighthouse_tag_202607/src/main.c` (add `#include`s, `uart_out_init()` call, `uart_out_push()` call, remove old `printk`)

- [ ] **Step 1: Add includes**

In `src/main.c`, add near the existing block of `#include`s (after `#include "robot_pose.h"`):

```c
#include <zephyr/logging/log.h>
#include "uart_out.h"
LOG_MODULE_REGISTER(main_app, CONFIG_LOG_DEFAULT_LEVEL);
```

- [ ] **Step 2: Initialize the UART module**

In `int main(void)` (currently line 112), immediately after `robot_pose_init();` (currently line 118) and before `ppi_init(TIMER_3);` (currently line 119), insert:

```c
	const struct device *uart0 = DEVICE_DT_GET(DT_NODELABEL(uart0));
	if (uart_out_init(uart0) != 0) {
		LOG_WRN("uart0 not ready; pose stream disabled");
		/* do not abort — BLE path still works */
	}
```

- [ ] **Step 3: Push pose into `uart_out` from the main loop**

In the `while (1)` loop (currently lines 179-202), find the `pose_q_push(&pkt);` call (currently line 193) and add the following line **immediately after it**:

```c
			uart_out_push(&pkt);
```

- [ ] **Step 4: Remove the legacy `printk` coordinate line**

In the same loop, delete the entire block:

```c
			uint16_t id = (uint16_t)((uint16_t)pkt.id_hi << 8) | pkt.id_lo;
			const char *m = (pkt.mode == 0x03) ? "3D" : "2D";
			printk("x=%.3f y=%.3f z=%.3f id=%u mode=%s\n",
			       (double)pkt.x_le, (double)pkt.y_le, (double)pkt.z_le,
			       id, m);
```

This block is now obsolete — `uart_out_push` produces the same data over UART.

Leave the other two `printk` calls (`ts4231_n%d init fail/OK` and `device_id=...`) intact: at the moment `CONFIG_PRINTK=n` they are no-ops, but removing them is out of scope for this change. They may be replaced by `LOG_*` calls in a follow-up.

- [ ] **Step 5: Build to confirm full compilation succeeds**

```bash
west build -b nrf52833dk_nrf52833 -p auto nrf52833_lighthouse_tag_202607/
```
Expected: build succeeds (modulo the pre-existing `ppi.c` ISR gap noted in spec §12 — if it surfaces here, stop and follow the spec's "do not bundle the fix" guidance).

- [ ] **Step 6: Commit**

```bash
git add nrf52833_lighthouse_tag_202607/src/main.c
git commit -m "feat(main): push pose to uart_out at 10 ms"
```

---

### Task 8: Update README with dual-output section

**Files:**
- Modify: `nrf52833_lighthouse_tag_202607/README.md`

- [ ] **Step 1: Append a "Dual Output (BLE + UART)" section**

Locate the end of the existing `README.md` and append the following section verbatim:

```markdown

## Dual Output (BLE + UART)

In addition to the ~50 Hz BLE broadcaster, the firmware emits a 100 Hz ASCII
pose stream on **UART0** at 115200 8N1.

Pin assignment on the nRF52833 DK:

| Signal | Pin   | Notes                |
|--------|-------|----------------------|
| UART0 TX | P0.06 | DK virtual COM RX    |
| UART0 RX | P0.08 | DK virtual COM TX    |

If your wiring differs, override the pinctrl block in the application
overlay (see `boards/nrf52833dk_nrf52833.overlay`).

### UART line format

One line per pose update, ASCII, `\r\n` terminated:

```
x=0.290 y=0.280 z=0.000 id=0x1234 mode=2D
```

- `x`, `y`, `z` are meters, three decimal places.
- `id` is the low 16 bits of `NRF_FICR->DEVICEADDR[0]`.
- `mode` is `2D` or `3D` (depends on `LIGHTHOUSE_MODE_3D`).

### Console

`printk` and the Zephyr console are disabled on UART0 — that line is owned
by the pose stream. For live debug logs, enable RTT by adding
`CONFIG_LOG_BACKEND_RTT=y` to `prj.conf` and using `LOG_INF` / `LOG_WRN`.
```

- [ ] **Step 2: Commit**

```bash
git add nrf52833_lighthouse_tag_202607/README.md
git commit -m "docs: describe dual BLE+UART0 pose output"
```

---

## Self-Review

**1. Spec coverage:**

| Spec section | Implementing task |
|---|---|
| §4 Module design (header) | Task 2 |
| §4 Module design (impl) | Task 3 |
| §4.3 Shared `lighthouse_pkt_t` header | Task 1 |
| §5 Devicetree overlay | Task 6 |
| §6 Kconfig changes | Task 5 |
| §7 `main.c` integration (init + push + remove printk) | Task 7 |
| §8 Build wiring | Task 4 |
| §11 Documentation updates | Task 8 |

**2. Placeholder scan:** No "TBD" / "implement later" / vague language. Every code block contains exact code.

**3. Type consistency:** `lighthouse_pkt_t` is defined once in Task 1 and consumed unchanged in Tasks 2/3/7. `uart_out_init` and `uart_out_push` signatures match between Tasks 2 and 3.

**4. Risk flagged:** Pre-existing `ppi.c` build break (spec §12) is called out at each `west build` step but not bundled into this plan, per the spec's guidance.