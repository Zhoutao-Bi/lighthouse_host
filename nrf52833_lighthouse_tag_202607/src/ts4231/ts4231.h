/**
 * @brief TS4231 Lighthouse sensor driver
 *
 * This module implements the TS4231 sensor driver including:
 * - GPIO pin configuration (D/E pins)
 * - State machine management (S0, SLEEP, WATCH, S3)
 * - Configuration read/write
 * - Bus state checking
 *
 * Ported from atom-robot firmware (BSD, UC Regents).
 * Original author: Tengfei Chang <tengfei.chang@gmail.com>, Nov 2021.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TS4231_H
#define TS4231_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>

#define S0_STATE         0x00
#define SLEEP_STATE      0x01
#define WATCH_STATE      0x02
#define S3_STATE         0x03
#define UNKNOWN_STATE    0x04

#define UNKNOWN          0x00
#define BUS_FAIL         0x01
#define VERIFY_FAIL      0x02
#define WATCH_FAIL       0x03
#define CONFIG_PASS      0x04

typedef struct {
	struct gpio_dt_spec e_spec;
	struct gpio_dt_spec d_spec;
	bool configured;
	bool is_lighthouse;
	uint8_t config_result;
	uint8_t origin_state;
	uint8_t current_state;
	uint32_t temp_in;
	uint8_t chip_state;
} ts4231_sensor_t;

int  ts4231_init(ts4231_sensor_t *sensor, uint32_t timeout_ms);
bool ts4231_is_lighthouse(const ts4231_sensor_t *sensor);

ts4231_sensor_t *ts4231_n1_handle(void);
ts4231_sensor_t *ts4231_n2_handle(void);
ts4231_sensor_t *ts4231_n3_handle(void);

bool ts4231_waitForLight(ts4231_sensor_t *sensor, uint32_t timeout_ms);
uint8_t ts4231_configDevice(ts4231_sensor_t *sensor);
uint16_t ts4231_readConfig(const ts4231_sensor_t *sensor);
void ts4231_writeConfig(ts4231_sensor_t *sensor, uint16_t config_val);
uint8_t ts4231_checkBus(const ts4231_sensor_t *sensor);
bool ts4231_goToSleep(ts4231_sensor_t *sensor);
bool ts4231_goToWatch(ts4231_sensor_t *sensor);

void ts4231_pinMode(const struct gpio_dt_spec *spec, uint8_t mode);
uint32_t ts4231_digitalRead(const struct gpio_dt_spec *spec);
void ts4231_digitalWrite(const struct gpio_dt_spec *spec, uint8_t output_mode);

void ts4231_set_pins(ts4231_sensor_t *sensor,
		     const struct gpio_dt_spec *e_spec,
		     const struct gpio_dt_spec *d_spec);
void ts4231_init_with_params(ts4231_sensor_t *sensor,
			     const struct gpio_dt_spec *e_spec,
			     const struct gpio_dt_spec *d_spec);
void ts4231_sensor_init(ts4231_sensor_t *sensor);

#endif /* TS4231_H */
