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

#include "timer.h"

#include <zephyr/kernel.h>
#include <hal/nrf_timer.h>

static NRF_TIMER_Type *timer_get_reg(timer_instance_t instance)
{
	switch (instance) {
	case TIMER_0: return NRF_TIMER0;
	case TIMER_1: return NRF_TIMER1;
	case TIMER_2: return NRF_TIMER2;
	case TIMER_3: return NRF_TIMER3;
	case TIMER_4: return NRF_TIMER4;
	default:     return NULL;
	}
}

void timer_init(timer_instance_t instance, uint8_t prescaler, uint8_t bitmode)
{
	NRF_TIMER_Type *timer;

	if (instance > TIMER_4) {
		return;
	}

	timer = timer_get_reg(instance);
	if (timer == NULL) {
		return;
	}

	nrf_timer_mode_set(timer, NRF_TIMER_MODE_TIMER);
	nrf_timer_bit_width_set(timer, (nrf_timer_bit_width_t)bitmode);
	nrf_timer_prescaler_set(timer, prescaler);
	nrf_timer_task_trigger(timer, NRF_TIMER_TASK_CLEAR);
}

void timer_start(timer_instance_t instance)
{
	NRF_TIMER_Type *timer = timer_get_reg(instance);

	if (timer == NULL) {
		return;
	}
	nrf_timer_task_trigger(timer, NRF_TIMER_TASK_START);
}

void timer_stop(timer_instance_t instance)
{
	NRF_TIMER_Type *timer = timer_get_reg(instance);

	if (timer == NULL) {
		return;
	}
	nrf_timer_task_trigger(timer, NRF_TIMER_TASK_STOP);
}

void timer_clear(timer_instance_t instance)
{
	NRF_TIMER_Type *timer = timer_get_reg(instance);

	if (timer == NULL) {
		return;
	}
	nrf_timer_task_trigger(timer, NRF_TIMER_TASK_CLEAR);
}

uint32_t timer_getCapturedValue(timer_instance_t instance, uint8_t capture_id)
{
	NRF_TIMER_Type *timer = timer_get_reg(instance);

	if (timer == NULL || capture_id >= 6) {
		return 0;
	}
	return nrf_timer_cc_get(timer, capture_id);
}

void timer_capture_now(timer_instance_t instance, uint8_t capture_id)
{
	NRF_TIMER_Type *timer = timer_get_reg(instance);

	if (timer == NULL || capture_id >= 6) {
		return;
	}
	nrf_timer_task_trigger(timer, (nrf_timer_task_t)(NRF_TIMER_TASK_CAPTURE0 + capture_id));
}
