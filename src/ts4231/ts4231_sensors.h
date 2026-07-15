/*
 * Copyright (c) Regents of the University of California.
 * All rights reserved.
 *
 * Ported to Zephyr / NCS v3.4.0 from atom-robot firmware.
 * Original author: Tengfei Chang <tengfei.chang@gmail.com>, Nov 2021
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TS4231_SENSORS_H
#define TS4231_SENSORS_H

#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

#include "timer.h"
#include "ts4231.h"

typedef struct {
	struct gpio_dt_spec e_spec;
	struct gpio_dt_spec d_spec;
} ts4231_sensor_config_t;

void ts4231_sensor_prepare(ts4231_sensor_t *sensor,
			   const ts4231_sensor_config_t *config);
bool ts4231_sensor_start_from_config(ts4231_sensor_t *sensor,
				     const ts4231_sensor_config_t *config);
void ts4231_sensor_attach_ppi(const ts4231_sensor_t *sensor,
			      timer_instance_t timer_instance);
void ts4231_sensor_attach_ppi_index(const ts4231_sensor_t *sensor,
				    timer_instance_t timer_instance,
				    uint8_t sensor_idx);

#endif /* TS4231_SENSORS_H */
