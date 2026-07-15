/*
 * Copyright (c) Regents of the University of California.
 * All rights reserved.
 *
 * Ported to Zephyr / NCS v3.4.0 from atom-robot firmware.
 * Original author: Cheng Wang <cwang199@connect.hkust-gz.edu.cn>, December 2023.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef POS_H
#define POS_H

#include <stdbool.h>
#include <stdint.h>

#define POS_MAX_SENSORS 3

typedef struct {
	double x;
	double y;
	double z;
} lighthouse_point;

typedef struct {
	double beta_deg;
	double alpha_deg;
} lighthouse_angles;

typedef struct {
	double yaw;
	double pitch;
	double roll;
} LH_Euler;

typedef struct {
	int valid;
	lighthouse_point position;
	LH_Euler attitude;
	double R[3][3];
} lighthouse_result;

void pos_init(void);
void pos_process_light_signal(uint8_t sensor_idx, uint32_t t_start,
			      uint32_t t_end, uint32_t duration);
int lighthouse_calibrate(const lighthouse_point world_points[],
			 const lighthouse_angles world_angles[],
			 lighthouse_result *out_result);
int lighthouse_get_position_simple(const lighthouse_result *calib_data,
				   double alpha_deg, double beta_deg,
				   lighthouse_point *out_pos);
int lighthouse_get_position_3d(const lighthouse_result *calib_a,
			      const lighthouse_result *calib_b,
			      double alpha_a_deg, double beta_a_deg,
			      double alpha_b_deg, double beta_b_deg,
			      lighthouse_point *out_pos);
float pos_get_A_X_theta_beta(uint8_t sensor_idx);
float pos_get_A_Y_theta_alpha(uint8_t sensor_idx);
const uint8_t *pos_get_data_xy(uint8_t sensor_idx);
const lighthouse_result *pos_get_base_station(void);
const lighthouse_point *pos_get_robot_pos(uint8_t sensor_idx);
int pos_get_sensor_position(uint8_t sensor_idx,
			    const lighthouse_result *calib_data,
			    lighthouse_point *out_pos);
uint8_t pos_get_sensor_count(void);

#endif /* POS_H */
