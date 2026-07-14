/*
 * Copyright (c) Regents of the University of California.
 * All rights reserved.
 *
 * Ported to Zephyr / NCS v3.4.0 from atom-robot firmware.
 * Original author: Tengfei Chang <tengfei.chang@gmail.com>, Nov 2021.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ts4231.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define MODE_INPUT     0
#define MODE_OUTPUT    1

#define OUTPUT_LOW     0
#define OUTPUT_HIGH    1

#define BUS_DRV_DLY    1
#define BUS_CHECK_DLY  500
#define SLEEP_RECOVERY 100

#define CFG_WORD       0x392B

static ts4231_sensor_t ts4231_n1_sensor = {
	.e_spec = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(ts4231_n1), e_gpios, 0),
	.d_spec = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(ts4231_n1), d_gpios, 0),
};

static ts4231_sensor_t ts4231_n2_sensor = {
	.e_spec = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(ts4231_n2), e_gpios, 0),
	.d_spec = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(ts4231_n2), d_gpios, 0),
};

static ts4231_sensor_t ts4231_n3_sensor = {
	.e_spec = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(ts4231_n3), e_gpios, 0),
	.d_spec = GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(ts4231_n3), d_gpios, 0),
};

ts4231_sensor_t *ts4231_n1_handle(void) { return &ts4231_n1_sensor; }
ts4231_sensor_t *ts4231_n2_handle(void) { return &ts4231_n2_sensor; }
ts4231_sensor_t *ts4231_n3_handle(void) { return &ts4231_n3_sensor; }

int ts4231_init(ts4231_sensor_t *sensor, uint32_t timeout_ms)
{
	if (sensor == NULL) {
		return -EINVAL;
	}

	int8_t repeat_config = 0;

	sensor->configured = false;
	sensor->is_lighthouse = false;
	sensor->config_result = 0x66;
	sensor->origin_state = 0x66;
	sensor->current_state = 0x66;
	sensor->temp_in = 0x01;
	sensor->chip_state = 0x99;

	gpio_pin_configure_dt(&sensor->e_spec, GPIO_INPUT);
	gpio_pin_configure_dt(&sensor->d_spec, GPIO_INPUT);

	if (!ts4231_waitForLight(sensor, timeout_ms)) {
		return -ETIMEDOUT;
	}
	sensor->is_lighthouse = true;

	sensor->origin_state = ts4231_checkBus(sensor);
	sensor->current_state = ts4231_checkBus(sensor);

	if (sensor->current_state != WATCH_STATE) {
		sensor->config_result = ts4231_configDevice(sensor);
		while ((sensor->current_state != WATCH_STATE) &&
		       (sensor->config_result != CONFIG_PASS) &&
		       (repeat_config < 10)) {
			repeat_config++;
			sensor->config_result = ts4231_configDevice(sensor);
			ts4231_goToWatch(sensor);
			sensor->current_state = ts4231_checkBus(sensor);
		}
	}

	return 0;
}

bool ts4231_is_lighthouse(const ts4231_sensor_t *sensor)
{
	if (sensor == NULL) {
		return false;
	}
	return sensor->is_lighthouse;
}

bool ts4231_waitForLight(ts4231_sensor_t *sensor, uint32_t timeout_ms)
{
	if (sensor == NULL) {
		return false;
	}

	int64_t deadline = (int64_t)k_uptime_get() + (int64_t)timeout_ms;
	bool light = false;
	bool exit_flag = false;

	if (ts4231_checkBus(sensor) == S0_STATE) {
		while (!exit_flag && (k_uptime_get() < deadline)) {
			if (gpio_pin_get_dt(&sensor->d_spec) != 0) {
				while (!exit_flag && (k_uptime_get() < deadline)) {
					if (gpio_pin_get_dt(&sensor->d_spec) == 0) {
						exit_flag = true;
						light = true;
					}
				}
			}
		}
	} else {
		light = true;
	}
	return light;
}

uint8_t ts4231_configDevice(ts4231_sensor_t *sensor)
{
	uint16_t config_val = CFG_WORD;
	uint8_t config_success = UNKNOWN;
	uint16_t readback;

	sensor->configured = false;
	ts4231_pinMode(&sensor->d_spec, MODE_INPUT);
	ts4231_pinMode(&sensor->e_spec, MODE_INPUT);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
	ts4231_pinMode(&sensor->e_spec, MODE_OUTPUT);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_pinMode(&sensor->d_spec, MODE_OUTPUT);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_pinMode(&sensor->e_spec, MODE_INPUT);
	ts4231_pinMode(&sensor->d_spec, MODE_INPUT);
	sensor->chip_state = ts4231_checkBus(sensor);
	if (sensor->chip_state == S3_STATE) {
		ts4231_writeConfig(sensor, config_val);
		readback = ts4231_readConfig(sensor);
		if (readback == config_val) {
			sensor->configured = SLEEP_STATE;
			if (ts4231_goToWatch(sensor)) {
				config_success = CONFIG_PASS;
			} else {
				config_success = WATCH_FAIL;
			}
		} else {
			config_success = VERIFY_FAIL;
		}
	} else {
		config_success = BUS_FAIL;
	}

	return config_success;
}

bool ts4231_goToSleep(ts4231_sensor_t *sensor)
{
	bool sleep_success;

	if (sensor->configured == false) {
		sleep_success = false;
	} else {
		switch (ts4231_checkBus(sensor)) {
		case S0_STATE:
			sleep_success = false;
			break;

		case SLEEP_STATE:
			sleep_success = true;
			break;

		case WATCH_STATE:
			ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
			ts4231_pinMode(&sensor->e_spec, MODE_OUTPUT);
			k_busy_wait(BUS_DRV_DLY);
			ts4231_pinMode(&sensor->e_spec, MODE_INPUT);
			k_busy_wait(BUS_DRV_DLY);

			if (ts4231_checkBus(sensor) == SLEEP_STATE) {
				sleep_success = true;
			} else {
				sleep_success = false;
			}
			break;
		case S3_STATE:
			sleep_success = false;
			break;
		default:
			sleep_success = false;
			break;
		}
	}
	return sleep_success;
}

bool ts4231_goToWatch(ts4231_sensor_t *sensor)
{
	bool watch_success;

	if (sensor->configured == false) {
		watch_success = false;
	} else {
		switch (ts4231_checkBus(sensor)) {
		case S0_STATE:
			watch_success = false;
			break;
		case SLEEP_STATE:
			ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
			ts4231_pinMode(&sensor->d_spec, MODE_OUTPUT);
			ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
			ts4231_pinMode(&sensor->e_spec, MODE_OUTPUT);
			ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
			ts4231_pinMode(&sensor->d_spec, MODE_INPUT);
			ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
			ts4231_pinMode(&sensor->e_spec, MODE_INPUT);
			k_busy_wait(SLEEP_RECOVERY);

			if (ts4231_checkBus(sensor) == WATCH_STATE) {
				watch_success = true;
			} else {
				watch_success = false;
			}
			break;

		case WATCH_STATE:
			watch_success = true;
			break;

		case S3_STATE:
			ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
			ts4231_pinMode(&sensor->e_spec, MODE_OUTPUT);
			ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
			ts4231_pinMode(&sensor->d_spec, MODE_OUTPUT);
			ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
			ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
			ts4231_pinMode(&sensor->d_spec, MODE_INPUT);
			ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
			ts4231_pinMode(&sensor->e_spec, MODE_INPUT);
			k_busy_wait(SLEEP_RECOVERY);

			if (ts4231_checkBus(sensor) == WATCH_STATE) {
				watch_success = true;
			} else {
				watch_success = false;
			}
			break;
		default:
			watch_success = false;
			break;
		}
	}
	return watch_success;
}

uint8_t ts4231_checkBus(const ts4231_sensor_t *sensor)
{
	uint8_t state;
	uint8_t e_state;
	uint8_t d_state;
	uint8_t s0_count = 0;
	uint8_t sleep_count = 0;
	uint8_t watch_count = 0;
	uint8_t s3_count = 0;

	for (uint8_t i = 0; i < 3; i++) {
		e_state = gpio_pin_get_dt(&sensor->e_spec);
		d_state = gpio_pin_get_dt(&sensor->d_spec);
		if (d_state == 1) {
			if (e_state == 1) {
				s3_count++;
			} else {
				sleep_count++;
			}
		} else {
			if (e_state == 1) {
				watch_count++;
			} else {
				s0_count++;
			}
		}
		k_busy_wait(BUS_CHECK_DLY);
	}
	if (sleep_count >= 2) {
		state = SLEEP_STATE;
	} else if (watch_count >= 2) {
		state = WATCH_STATE;
	} else if (s3_count >= 2) {
		state = S3_STATE;
	} else if (s0_count >= 2) {
		state = S0_STATE;
	} else {
		state = UNKNOWN_STATE;
	}

	return state;
}

void ts4231_pinMode(const struct gpio_dt_spec *spec, uint8_t mode)
{
	if (mode == MODE_OUTPUT) {
		gpio_pin_configure_dt(spec, GPIO_OUTPUT);
	} else {
		gpio_pin_configure_dt(spec, GPIO_INPUT);
	}
}

uint32_t ts4231_digitalRead(const struct gpio_dt_spec *spec)
{
	return gpio_pin_get_dt(spec) ? 1UL : 0UL;
}

void ts4231_digitalWrite(const struct gpio_dt_spec *spec, uint8_t output_mode)
{
	gpio_pin_set_dt(spec, output_mode == OUTPUT_HIGH ? 1 : 0);
	k_busy_wait(1);
}

void ts4231_writeConfig(ts4231_sensor_t *sensor, uint16_t config_val)
{
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
	ts4231_pinMode(&sensor->e_spec, MODE_OUTPUT);
	ts4231_pinMode(&sensor->d_spec, MODE_OUTPUT);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
	k_busy_wait(BUS_DRV_DLY);

	for (uint8_t i = 0; i < 15; i++) {
		config_val = config_val << 1;
		if ((config_val & 0x8000) > 0) {
			ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
		} else {
			ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
		}

		k_busy_wait(BUS_DRV_DLY);
		ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
		k_busy_wait(BUS_DRV_DLY);
		ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
		k_busy_wait(BUS_DRV_DLY);
	}
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_pinMode(&sensor->e_spec, MODE_INPUT);
	ts4231_pinMode(&sensor->d_spec, MODE_INPUT);
}

uint16_t ts4231_readConfig(const ts4231_sensor_t *sensor)
{
	uint16_t readback = 0x0000;

	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
	ts4231_pinMode(&sensor->e_spec, MODE_OUTPUT);
	ts4231_pinMode(&sensor->d_spec, MODE_OUTPUT);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_pinMode(&sensor->d_spec, MODE_INPUT);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
	k_busy_wait(BUS_DRV_DLY);

	for (uint8_t i = 0; i < 14; i++) {
		ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
		k_busy_wait(BUS_DRV_DLY);
		readback = (readback << 1) | (ts4231_digitalRead(&sensor->d_spec) & 0x0001);
		ts4231_digitalWrite(&sensor->e_spec, OUTPUT_LOW);
		k_busy_wait(BUS_DRV_DLY);
	}

	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_LOW);
	ts4231_pinMode(&sensor->d_spec, MODE_OUTPUT);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->e_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_digitalWrite(&sensor->d_spec, OUTPUT_HIGH);
	k_busy_wait(BUS_DRV_DLY);
	ts4231_pinMode(&sensor->e_spec, MODE_INPUT);
	ts4231_pinMode(&sensor->d_spec, MODE_INPUT);

	return readback;
}

void ts4231_set_pins(ts4231_sensor_t *sensor,
		     const struct gpio_dt_spec *e_spec,
		     const struct gpio_dt_spec *d_spec)
{
	if (sensor == NULL) {
		return;
	}
	sensor->e_spec = *e_spec;
	sensor->d_spec = *d_spec;
}

void ts4231_init_with_params(ts4231_sensor_t *sensor,
			     const struct gpio_dt_spec *e_spec,
			     const struct gpio_dt_spec *d_spec)
{
	ts4231_set_pins(sensor, e_spec, d_spec);
	ts4231_init(sensor, 100);
}

void ts4231_sensor_init(ts4231_sensor_t *sensor)
{
	if (sensor == NULL) {
		return;
	}
	ts4231_init(sensor, 100);
}
