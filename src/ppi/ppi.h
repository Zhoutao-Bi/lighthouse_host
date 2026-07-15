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

#ifndef PPI_H
#define PPI_H

#include <zephyr/drivers/gpio.h>
#include <stdint.h>

#include "timer.h"

#define PPI_MAX_SENSORS 3

typedef void (*ppi_light_signal_cb_t)(uint32_t t_start, uint32_t t_end,
				      uint32_t duration);
typedef void (*ppi_light_signal_ex_cb_t)(uint8_t sensor_idx,
					uint32_t t_start, uint32_t t_end,
					uint32_t duration);

void ppi_gpiote_init(const struct gpio_dt_spec *spec);
void ppi_gpiote_init_sensor(uint8_t sensor_idx,
			    const struct gpio_dt_spec *spec);
void ppi_init(timer_instance_t timer_instance);
void ppi_init_multi(timer_instance_t timer_instance, uint8_t sensor_count);
void ppi_set_light_signal_callback(ppi_light_signal_cb_t cb);
void ppi_set_light_signal_ex_callback(ppi_light_signal_ex_cb_t cb);

#endif /* PPI_H */
