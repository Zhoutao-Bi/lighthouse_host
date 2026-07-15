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