/*
 * Copyright (c) Regents of the University of California.
 * All rights reserved.
 *
 * Ported to Zephyr / NCS v3.4.0 from atom-robot firmware.
 * Original author: unknown (robot_pose.c).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ROBOT_POSE_H
#define ROBOT_POSE_H

#include "pos.h"

#define ROBOT_POSE_SENSOR_RADIUS_MM 12.729
#define ROBOT_POSE_MIN_VALID_SENSORS 2u

#define ROBOT_POSE_SENSOR0_ANGLE_DEG 330.0
#define ROBOT_POSE_SENSOR1_ANGLE_DEG 210.0
#define ROBOT_POSE_SENSOR2_ANGLE_DEG 90.0

typedef struct {
	bool valid;
	uint8_t valid_sensor_count;
	uint8_t valid_sensor_mask;
	lighthouse_point position;
	float heading_deg;
} robot_pose_t;

typedef struct {
	bool valid;
	lighthouse_point position;
	float heading_deg;
} robot_pose_calibrated_t;

void robot_pose_init(void);
void robot_pose_set_calib_data(lighthouse_result *calib_data);
int robot_pose_update(void);
const robot_pose_t *robot_pose_get(void);
const lighthouse_point *robot_pose_get_position(void);
bool robot_pose_get_heading_deg(float *heading_deg);
bool robot_pose_is_valid(void);
bool robot_pose_set_reference(const lighthouse_point *position, float heading_deg);
bool robot_pose_set_reference_from_current(void);
void robot_pose_clear_reference(void);
bool robot_pose_has_reference(void);
bool robot_pose_get_calibrated(robot_pose_calibrated_t *out_pose);

#endif /* ROBOT_POSE_H */
