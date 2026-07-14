#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#include "bt_scan.h"

static bt_scan_rx_fn_t rx_callback;

struct scan_ctx {
	int8_t rssi;
	const bt_addr_le_t *addr;
};

static bool mfg_data_cb(struct bt_data *data, void *user_data)
{
	struct scan_ctx *ctx = user_data;

	if (data->type != BT_DATA_MANUFACTURER_DATA) {
		return true;
	}
	if (data->data_len < 2) {
		return true;
	}
	if (rx_callback == NULL) {
		return true;
	}

	rx_callback(&data->data[2], data->data_len - 2, ctx->rssi, ctx->addr);
	return true;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
		    struct net_buf_simple *ad)
{
	struct scan_ctx ctx = { .rssi = rssi, .addr = addr };
	bt_data_parse(ad, mfg_data_cb, &ctx);
}

int bt_scan_register_callback(bt_scan_rx_fn_t fn)
{
	rx_callback = fn;
	return 0;
}

int bt_scan_start(void)
{
	static const struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_FILTER_DUPLICATE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	return bt_le_scan_start(&scan_param, scan_cb);
}
