/*
 * Copyright (c) Regents of the University of California.
 * All rights reserved.
 *
 * Ported to Zephyr / NCS v3.4.0 from atom-robot firmware.
 * Original author: Manjiang Cao <manjiang19@hkust-gz.edu.cn> Atomic. Nov, 2023
 *                 Tengfei Chang <tengfeichang@hkust-gz.edu.cn> Atomic. Nov, 2023
 *                 Cheng Wang <cwang199@connect.hkust-gz.edu.cn>, Dec 2023
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PPI_TIMER_H
#define PPI_TIMER_H

#include <stdint.h>

#define TIMER_BITMODE_16BIT   0
#define TIMER_BITMODE_8BIT    1
#define TIMER_BITMODE_24BIT   2
#define TIMER_BITMODE_32BIT   3

typedef enum {
	TIMER_0 = 0,
	TIMER_1 = 1,
	TIMER_2 = 2,
	TIMER_3 = 3,
	TIMER_4 = 4,
} timer_instance_t;

void     timer_init(timer_instance_t instance, uint8_t prescaler, uint8_t bitmode);
void     timer_start(timer_instance_t instance);
void     timer_stop(timer_instance_t instance);
void     timer_clear(timer_instance_t instance);
uint32_t timer_getCapturedValue(timer_instance_t instance, uint8_t capture_id);
void     timer_capture_now(timer_instance_t instance, uint8_t capture_id);

#endif /* PPI_TIMER_H */
