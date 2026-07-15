#ifndef BT_ADVERTISE_H
#define BT_ADVERTISE_H

#include <stddef.h>
#include <stdint.h>

#define BT_ADV_REFRESH_PERIOD_MS 20
#define BT_ADV_MAX_PAYLOAD       64

typedef size_t (*bt_adv_payload_fn_t)(uint8_t *buf, size_t max_len);

int  bt_advertise_set_payload_provider(bt_adv_payload_fn_t fn);
int  bt_advertise_start(void);
void bt_advertise_notify_rx(void);

#endif
