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

#include "ts4231_sensors.h"

#include <string.h>

#include "ppi.h"

typedef struct {
	bool timer_started;
	timer_instance_t timer_instance;
	uint8_t highest_sensor_count;
} ts4231_sensors_vars_t;

static ts4231_sensors_vars_t ts4231_sensors_vars = {
	.timer_started = false,
	.timer_instance = TIMER_3,
	.highest_sensor_count = 0,
};

static void ts4231_sensors_ensure_timer(timer_instance_t timer_instance)
{
	if (!ts4231_sensors_vars.timer_started ||
	    ts4231_sensors_vars.timer_instance != timer_instance) {
		timer_init(timer_instance, 0, TIMER_BITMODE_32BIT);
		timer_start(timer_instance);
		ts4231_sensors_vars.timer_started = true;
		ts4231_sensors_vars.timer_instance = timer_instance;
		ts4231_sensors_vars.highest_sensor_count = 0;
	}
}

static void ts4231_sensors_update_count(uint8_t sensor_idx)
{
	uint8_t required_count = (uint8_t)(sensor_idx + 1);

	if (required_count > ts4231_sensors_vars.highest_sensor_count) {
		ts4231_sensors_vars.highest_sensor_count = required_count;
	}
}

void ts4231_sensor_prepare(ts4231_sensor_t *sensor,
			   const ts4231_sensor_config_t *config)
{
	if (sensor == NULL || config == NULL) {
		return;
	}

	memset(sensor, 0, sizeof(ts4231_sensor_t));
	sensor->e_spec = config->e_spec;
	sensor->d_spec = config->d_spec;
}

bool ts4231_sensor_start_from_config(ts4231_sensor_t *sensor,
				     const ts4231_sensor_config_t *config)
{
	if (sensor == NULL || config == NULL) {
		return false;
	}

	ts4231_sensor_prepare(sensor, config);
	ts4231_sensor_init(sensor);
	ts4231_sensor_attach_ppi(sensor, TIMER_3);

	if (!sensor->is_lighthouse) {
		return false;
	}

	return (sensor->current_state == WATCH_STATE);
}

void ts4231_sensor_attach_ppi(const ts4231_sensor_t *sensor,
			      timer_instance_t timer_instance)
{
	ts4231_sensor_attach_ppi_index(sensor, timer_instance, 0);
}

void ts4231_sensor_attach_ppi_index(const ts4231_sensor_t *sensor,
				    timer_instance_t timer_instance,
				    uint8_t sensor_idx)
{
	if (sensor == NULL) {
		return;
	}
	if (sensor_idx >= PPI_MAX_SENSORS) {
		return;
	}

	ts4231_sensors_ensure_timer(timer_instance);
	ppi_gpiote_init_sensor(sensor_idx, &sensor->e_spec);
	ts4231_sensors_update_count(sensor_idx);
	ppi_init_multi(timer_instance, ts4231_sensors_vars.highest_sensor_count);
}
