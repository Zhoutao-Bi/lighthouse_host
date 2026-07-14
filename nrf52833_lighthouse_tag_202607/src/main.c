#include <zephyr/kernel.h>
#include <string.h>

#include "led.h"
#include "bt_advertise.h"
#include "bt_scan.h"
#include "ts4231.h"
#include "ts4231_sensors.h"
#include "ppi.h"

static size_t counter_provider(uint8_t *buf, size_t max_len)
{
	const size_t need = 2 + sizeof(uint32_t);

	if (max_len < need) {
		return 0;
	}

	static uint32_t counter;

	buf[0] = 0xFF;
	buf[1] = 0xFF;
	memcpy(&buf[2], &counter, sizeof(counter));
	counter++;

	return need;
}

static void on_rx(const uint8_t *data, size_t len, int8_t rssi,
		  const bt_addr_le_t *addr)
{
	if (addr != NULL) {
		printk("RX %02x:%02x:%02x:%02x:%02x:%02x rssi=%d len=%zu\n",
		       addr->a.val[5], addr->a.val[4], addr->a.val[3],
		       addr->a.val[2], addr->a.val[1], addr->a.val[0],
		       rssi, len);
	}
	bt_advertise_notify_rx();
}

static void on_pulse(uint8_t sensor_idx, uint32_t t_start, uint32_t t_end,
		     uint32_t duration)
{
	ARG_UNUSED(sensor_idx);
	ARG_UNUSED(t_start);
	ARG_UNUSED(t_end);
	ARG_UNUSED(duration);
}

int main(void)
{
	led_init();
	led_set_mode(LED_MODE_INIT);

	ts4231_init();
	if (!ts4231_is_lighthouse()) {
		led_set_mode(LED_MODE_ERROR);
		return -ENODEV;
	}

	ts4231_sensor_attach_ppi_index(ts4231_default_handle(), TIMER_3, 0);
	ppi_set_light_signal_ex_callback(on_pulse);

	int err = bt_enable(NULL);
	if (err) {
		led_set_mode(LED_MODE_ERROR);
		return err;
	}

	bt_advertise_set_payload_provider(counter_provider);
	err = bt_advertise_start();
	if (err) {
		led_set_mode(LED_MODE_ERROR);
		return err;
	}

	bt_scan_register_callback(on_rx);
	err = bt_scan_start();
	if (err) {
		led_set_mode(LED_MODE_ERROR);
		return err;
	}

	return 0;
}
