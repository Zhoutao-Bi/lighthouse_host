#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <nrfx.h>
#include <string.h>

#include "led.h"
#include "bt_advertise.h"
#include "bt_scan.h"
#include "ts4231.h"
#include "ts4231_sensors.h"
#include "ppi.h"
#include "pos.h"
#include "robot_pose.h"
#include "lighthouse_config.h"

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

typedef struct __attribute__((packed)) {
	uint8_t mode;
	uint8_t id_lo;
	uint8_t id_hi;
	float x_le;
	float y_le;
	float z_le;
} lighthouse_pkt_t;

#define POSE_Q_SIZE 4
static struct {
	lighthouse_pkt_t buf[POSE_Q_SIZE];
	atomic_t head;
	atomic_t tail;
} pose_q;

static void pose_q_push(const lighthouse_pkt_t *p)
{
	uint32_t tail = (uint32_t)atomic_get(&pose_q.tail);
	pose_q.buf[tail % POSE_Q_SIZE] = *p;
	atomic_inc(&pose_q.tail);

	uint32_t head = (uint32_t)atomic_get(&pose_q.head);
	if ((uint32_t)(atomic_get(&pose_q.tail) - head) >= POSE_Q_SIZE) {
		atomic_inc(&pose_q.head);
	}
}

static bool pose_q_pop(lighthouse_pkt_t *out)
{
	uint32_t tail = (uint32_t)atomic_get(&pose_q.tail);
	uint32_t head = (uint32_t)atomic_get(&pose_q.head);

	if (head >= tail) {
		return false;
	}
	*out = pose_q.buf[head % POSE_Q_SIZE];
	atomic_inc(&pose_q.head);
	return true;
}

static uint16_t get_device_id(void)
{
	return (uint16_t)(NRF_FICR->DEVICEADDR[0] & 0xFFFFu);
}

static size_t pose_provider(uint8_t *buf, size_t max_len)
{
	if (max_len < sizeof(lighthouse_pkt_t) + 2) {
		return 0;
	}

	lighthouse_pkt_t pkt;
	if (!pose_q_pop(&pkt)) {
		return 0;
	}

	buf[0] = 0xFF;
	buf[1] = 0xFF;
	memcpy(&buf[2], &pkt, sizeof(lighthouse_pkt_t));
	return sizeof(lighthouse_pkt_t) + 2;
}

static void on_rx(const uint8_t *data, size_t len, int8_t rssi,
		  const bt_addr_le_t *addr)
{
	ARG_UNUSED(data);
	ARG_UNUSED(len);
	ARG_UNUSED(rssi);
	ARG_UNUSED(addr);
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
	ppi_init(TIMER_3);

	ts4231_sensor_t *handles[3] = {
		ts4231_n1_handle(),
		ts4231_n2_handle(),
		ts4231_n3_handle(),
	};

	uint8_t valid_sensors = 0;
	for (int i = 0; i < 3; i++) {
		int err = ts4231_init(handles[i], 50);
		if (err) {
			printk("ts4231_n%d init fail: %d\n", i + 1, err);
			continue;
		}
		valid_sensors++;
		ts4231_sensor_attach_ppi_index(handles[i], TIMER_3, (uint8_t)i);
		printk("ts4231_n%d init OK\n", i + 1);
	}

	if (valid_sensors == 0) {
		led_set_mode(LED_MODE_ERROR);
		return -ENODEV;
	}

	if (lighthouse_calibrate(cal_pos, cal_angles, &calib_result) == 0) {
		robot_pose_set_calib_data(&calib_result);
	}

	int err = bt_enable(NULL);
	if (err) {
		led_set_mode(LED_MODE_ERROR);
		return err;
	}

	uint16_t dev_id = get_device_id();
#if LIGHTHOUSE_MODE_3D
	const char *mode_str = "3D";
	uint8_t mode_flag = 0x03;
#else
	const char *mode_str = "2D";
	uint8_t mode_flag = 0x01;
#endif
	printk("device_id=0x%04x mode=%s sensors=%u\n",
	       dev_id, mode_str, valid_sensors);

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
			lighthouse_pkt_t pkt;

			pkt.mode = mode_flag;
			pkt.id_lo = (uint8_t)(dev_id & 0xFFu);
			pkt.id_hi = (uint8_t)((dev_id >> 8) & 0xFFu);
			pkt.x_le = (float)pose->position.x;
			pkt.y_le = (float)pose->position.y;
			pkt.z_le = (float)pose->position.z;

			pose_q_push(&pkt);

			uint16_t id = (uint16_t)((uint16_t)pkt.id_hi << 8) | pkt.id_lo;
			const char *m = (pkt.mode == 0x03) ? "3D" : "2D";
			printk("x=%.3f y=%.3f z=%.3f id=%u mode=%s\n",
			       (double)pkt.x_le, (double)pkt.y_le, (double)pkt.z_le,
			       id, m);
		}
		k_msleep(10);
	}

	return 0;
}
