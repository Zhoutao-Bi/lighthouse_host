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

#include "ppi.h"

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <hal/nrf_ppi.h>
#include <hal/nrf_gpiote.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_gpio.h>
#include <zephyr/sys/printk.h>

ISR_DIRECT_DECLARE(ppi_gpiote_isr);

typedef struct {
	ppi_light_signal_cb_t light_signal_cb;
	ppi_light_signal_ex_cb_t light_signal_ex_cb;
	timer_instance_t timer_instance;
	uint8_t sensor_count;
	uint8_t flag_start[PPI_MAX_SENSORS];
	uint32_t t_0_start[PPI_MAX_SENSORS];
} ppi_vars_t;

static ppi_vars_t ppi_vars;

static int ppi_port_from_spec(const struct gpio_dt_spec *spec)
{
	if (spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio0))) {
		return 0;
	}
	if (spec->port == DEVICE_DT_GET(DT_NODELABEL(gpio1))) {
		return 1;
	}
	return -1;
}

void ppi_gpiote_init(const struct gpio_dt_spec *spec)
{
	ppi_gpiote_init_sensor(0, spec);
}

void ppi_gpiote_init_sensor(uint8_t sensor_idx, const struct gpio_dt_spec *spec)
{
	if (sensor_idx >= PPI_MAX_SENSORS) {
		return;
	}
	if (spec == NULL || spec->port == NULL) {
		return;
	}

	if (ppi_vars.sensor_count == 0) {
		ppi_vars.timer_instance = TIMER_3;
		ppi_vars.sensor_count = 1;
	}
	if (sensor_idx + 1 > ppi_vars.sensor_count) {
		ppi_vars.sensor_count = sensor_idx + 1;
	}

	uint8_t falling_ch = (uint8_t)(2 * sensor_idx);
	uint8_t rising_ch = (uint8_t)(falling_ch + 1);
	uint8_t port = (uint8_t)ppi_port_from_spec(spec);
	uint8_t psel = (uint8_t)spec->pin;
	uint32_t encoded_pin = (uint32_t)((port << 5) | psel);

	nrf_gpiote_event_configure(NRF_GPIOTE, falling_ch, encoded_pin,
				   NRF_GPIOTE_POLARITY_HITOLO);
	nrf_gpiote_event_configure(NRF_GPIOTE, rising_ch, encoded_pin,
				   NRF_GPIOTE_POLARITY_LOTOHI);

	nrf_gpiote_event_enable(NRF_GPIOTE, falling_ch);
	nrf_gpiote_event_enable(NRF_GPIOTE, rising_ch);

	irq_enable(GPIOTE_IRQn);
}

void ppi_init(timer_instance_t timer_instance)
{
	IRQ_DIRECT_CONNECT(GPIOTE_IRQn, 0, ppi_gpiote_isr, 0);
	ppi_init_multi(timer_instance, 1);
}

void ppi_init_multi(timer_instance_t timer_instance, uint8_t sensor_count)
{
	ppi_vars.timer_instance = timer_instance;
	if (sensor_count == 0) {
		sensor_count = 1;
	}
	if (sensor_count > PPI_MAX_SENSORS) {
		sensor_count = PPI_MAX_SENSORS;
	}
	ppi_vars.sensor_count = sensor_count;

	NRF_TIMER_Type *timer;
	switch (timer_instance) {
	case TIMER_0: timer = NRF_TIMER0; break;
	case TIMER_1: timer = NRF_TIMER1; break;
	case TIMER_2: timer = NRF_TIMER2; break;
	case TIMER_3: timer = NRF_TIMER3; break;
	case TIMER_4: timer = NRF_TIMER4; break;
	default:     timer = NULL;   break;
	}
	if (timer == NULL) {
		return;
	}

	for (uint8_t idx = 0; idx < ppi_vars.sensor_count; idx++) {
		uint8_t gpiote_falling = (uint8_t)(2 * idx);
		uint8_t gpiote_rising = (uint8_t)(gpiote_falling + 1);
		uint8_t timer_cc_falling = gpiote_falling;
		uint8_t timer_cc_rising = gpiote_rising;
		uint8_t ppi_ch_falling = gpiote_falling;
		uint8_t ppi_ch_rising = gpiote_rising;

		nrf_ppi_channel_endpoint_setup(
			NRF_PPI, ppi_ch_falling,
			(uint32_t)nrf_gpiote_event_address_get(NRF_GPIOTE, gpiote_falling),
			(uint32_t)&timer->TASKS_CAPTURE[timer_cc_falling]);

		nrf_ppi_channel_endpoint_setup(
			NRF_PPI, ppi_ch_rising,
			(uint32_t)nrf_gpiote_event_address_get(NRF_GPIOTE, gpiote_rising),
			(uint32_t)&timer->TASKS_CAPTURE[timer_cc_rising]);

		nrf_ppi_channel_enable(NRF_PPI, ppi_ch_falling);
		nrf_ppi_channel_enable(NRF_PPI, ppi_ch_rising);
	}
}

void ppi_set_light_signal_callback(ppi_light_signal_cb_t cb)
{
	ppi_vars.light_signal_cb = cb;
}

void ppi_set_light_signal_ex_callback(ppi_light_signal_ex_cb_t cb)
{
	ppi_vars.light_signal_ex_cb = cb;
}

static inline int ppi_gpiote_isr_body(void)
{
	for (uint8_t sensor_idx = 0; sensor_idx < ppi_vars.sensor_count; sensor_idx++) {
		uint8_t falling_ch = (uint8_t)(2 * sensor_idx);
		uint8_t rising_ch = (uint8_t)(falling_ch + 1);

		if (nrf_gpiote_event_check(NRF_GPIOTE, (nrf_gpiote_event_t)falling_ch) &&
		    (nrf_gpiote_int_enable_check(NRF_GPIOTE, 1UL << falling_ch) != 0)) {
			nrf_gpiote_event_clear(NRF_GPIOTE, falling_ch);

			if (ppi_vars.flag_start[sensor_idx] == 0) {
				ppi_vars.t_0_start[sensor_idx] =
					timer_getCapturedValue(ppi_vars.timer_instance, falling_ch);
				ppi_vars.flag_start[sensor_idx] = 1;
			}
		}

		if (nrf_gpiote_event_check(NRF_GPIOTE, (nrf_gpiote_event_t)rising_ch) &&
		    (nrf_gpiote_int_enable_check(NRF_GPIOTE, 1UL << rising_ch) != 0)) {
			nrf_gpiote_event_clear(NRF_GPIOTE, rising_ch);

			if (ppi_vars.flag_start[sensor_idx] == 1) {
				uint32_t t_0_end =
					timer_getCapturedValue(ppi_vars.timer_instance, rising_ch);
				ppi_vars.flag_start[sensor_idx] = 0;
				uint32_t duration = t_0_end - ppi_vars.t_0_start[sensor_idx];

				if (ppi_vars.light_signal_ex_cb != NULL) {
					ppi_vars.light_signal_ex_cb(sensor_idx,
								      ppi_vars.t_0_start[sensor_idx],
								      t_0_end, duration);
				}
				if (ppi_vars.light_signal_cb != NULL) {
					ppi_vars.light_signal_cb(ppi_vars.t_0_start[sensor_idx],
								  t_0_end, duration);
				}
			}
		}
	}

	return 0;
}
