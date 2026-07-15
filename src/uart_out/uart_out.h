#pragma once
#include <zephyr/kernel.h>
#include "../lighthouse_pkt.h"

int  uart_out_init(const struct device *uart_dev);
void uart_out_push(const lighthouse_pkt_t *pkt);