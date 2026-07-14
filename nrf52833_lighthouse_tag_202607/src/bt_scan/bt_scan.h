#ifndef BT_SCAN_H
#define BT_SCAN_H

#include <stddef.h>
#include <stdint.h>
#include <zephyr/bluetooth/bluetooth.h>

typedef void (*bt_scan_rx_fn_t)(const uint8_t *data, size_t len,
				int8_t rssi, const bt_addr_le_t *addr);

int  bt_scan_register_callback(bt_scan_rx_fn_t fn);
int  bt_scan_start(void);

#endif
