#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "bt_advertise.h"
#include "../led/led.h"

static bt_adv_payload_fn_t payload_fn;
static bool adv_active;

static void adv_work_handler(struct k_work *w);
K_WORK_DEFINE(adv_work, adv_work_handler);

static void adv_timer_handler(struct k_timer *t);
K_TIMER_DEFINE(adv_timer, adv_timer_handler, NULL);

static void adv_work_handler(struct k_work *w)
{
	if (!adv_active || payload_fn == NULL) {
		return;
	}

	uint8_t buf[BT_ADV_MAX_PAYLOAD];
	size_t len = payload_fn(buf, sizeof(buf));

	if (len < 3) {
		return;
	}

	struct bt_data ad = {
		.type = BT_DATA_MANUFACTURER_DATA,
		.data_len = len,
		.data = buf,
	};

	int err = bt_le_adv_update_data(&ad, 1, NULL, 0);
	if (err && err != -EAGAIN) {
	}
}

static void adv_timer_handler(struct k_timer *t)
{
	k_work_submit(&adv_work);
}

int bt_advertise_set_payload_provider(bt_adv_payload_fn_t fn)
{
	payload_fn = fn;
	return 0;
}

int bt_advertise_start(void)
{
	static const struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_1,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_1,
		.options = BT_LE_ADV_OPT_NONE,
	};

	static const uint8_t flags_bytes[] = {
		BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR,
	};
	static const struct bt_data flags_ad = {
		.type = BT_DATA_FLAGS,
		.data_len = sizeof(flags_bytes),
		.data = flags_bytes,
	};

	int err = bt_le_adv_start(&adv_param, &flags_ad, 1, NULL, 0);
	if (err) {
		return err;
	}

	adv_active = true;
	k_timer_start(&adv_timer, K_MSEC(BT_ADV_REFRESH_PERIOD_MS),
		      K_MSEC(BT_ADV_REFRESH_PERIOD_MS));

	led_set_mode(LED_MODE_BROADCASTING);
	return 0;
}

void bt_advertise_notify_rx(void)
{
	led_notify_rx();
}
