#include <zephyr/kernel.h>
#include <string.h>

#include "led.h"
#include "bt_advertise.h"
#include "bt_scan.h"
#include "ts4231.h"
#include "ts4231_sensors.h"
#include "ppi.h"
#include "pos.h"
#include "robot_pose.h"

static const lighthouse_point cal_pos[] = {
	{0.0, 0.0, 0.0},
	{594.0, 0.0, 0.0},
	{594.0, 420.0, 0.0}
};

static const lighthouse_angles cal_angles[] = {
	{82.556, 74.598},
	{88.518, 87.382},
	{81.198, 91.95}
};

static lighthouse_result calib_result;

static size_t pose_provider(uint8_t *buf, size_t max_len)
{
	const size_t need = 2 + 1 + 1 + 1 + 1 + 8 * 3 + 4;

	if (max_len < need) {
		return 0;
	}

	const robot_pose_t *pose = robot_pose_get();

	buf[0] = 0xFF;
	buf[1] = 0xFF;
	buf[2] = pose->valid ? 1u : 0u;
	buf[3] = pose->valid_sensor_count;
	buf[4] = pose->valid_sensor_mask;
	buf[5] = 0;
	memcpy(&buf[6], &pose->position.x, sizeof(double));
	memcpy(&buf[14], &pose->position.y, sizeof(double));
	memcpy(&buf[22], &pose->position.z, sizeof(double));
	memcpy(&buf[30], &pose->heading_deg, sizeof(float));

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

	pos_init();
	robot_pose_init();

	ts4231_init();
	if (!ts4231_is_lighthouse()) {
		led_set_mode(LED_MODE_ERROR);
		return -ENODEV;
	}

	ts4231_sensor_attach_ppi_index(ts4231_default_handle(), TIMER_3, 0);

	if (lighthouse_calibrate(cal_pos, cal_angles, &calib_result) == 0) {
		robot_pose_set_calib_data(&calib_result);
	}

	int err = bt_enable(NULL);
	if (err) {
		led_set_mode(LED_MODE_ERROR);
		return err;
	}

	bt_advertise_set_payload_provider(pose_provider);
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

	while (1) {
		robot_pose_update();
		const robot_pose_t *pose = robot_pose_get();

		if (pose->valid) {
			float heading;
			if (robot_pose_get_heading_deg(&heading)) {
				printk("pose x=%.1f y=%.1f z=%.1f h=%.1f sensors=%u mask=0x%x\n",
				       pose->position.x, pose->position.y, pose->position.z,
				       (double)heading, pose->valid_sensor_count,
				       pose->valid_sensor_mask);
			}
		}
		k_msleep(100);
	}

	return 0;
}
