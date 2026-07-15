/*
 * Copyright (c) Regents of the University of California.
 * All rights reserved.
 *
 * Ported to Zephyr / NCS v3.4.0 from atom-robot firmware.
 * Original author: unknown (robot_pose.c).
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "robot_pose.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
	robot_pose_t pose;
	robot_pose_t reference_pose;
	robot_pose_calibrated_t calibrated_pose;
	lighthouse_point sensor_positions[POS_MAX_SENSORS];
	lighthouse_result *calib_data;
} robot_pose_vars_t;

static robot_pose_vars_t robot_pose_vars;

static lighthouse_point robot_pose_get_local_sensor_point(uint8_t sensor_idx);
static float robot_pose_normalize_heading(float heading_deg);
static float robot_pose_normalize_relative_heading(float heading_deg);
static void robot_pose_update_calibrated_pose(void);

void robot_pose_init(void)
{
	memset(&robot_pose_vars, 0, sizeof(robot_pose_vars));
}

void robot_pose_set_calib_data(lighthouse_result *calib_data)
{
	robot_pose_vars.calib_data = calib_data;
}

int robot_pose_update(void)
{
	lighthouse_point local_points[POS_MAX_SENSORS];
	lighthouse_point world_points[POS_MAX_SENSORS];
	lighthouse_point local_mean = {0.0, 0.0, 0.0};
	lighthouse_point world_mean = {0.0, 0.0, 0.0};
	double cross_sum = 0.0;
	double dot_sum = 0.0;
	uint8_t valid_count = 0;
	uint8_t valid_mask = 0;

	if (robot_pose_vars.calib_data == NULL ||
	    !robot_pose_vars.calib_data->valid) {
		robot_pose_vars.pose.valid = false;
		robot_pose_vars.pose.valid_sensor_count = 0;
		robot_pose_vars.pose.valid_sensor_mask = 0;
		robot_pose_vars.calibrated_pose.valid = false;
		return -1;
	}

	for (uint8_t sensor_idx = 0; sensor_idx < POS_MAX_SENSORS; sensor_idx++) {
		lighthouse_point sensor_position;

		if (pos_get_sensor_position(sensor_idx, robot_pose_vars.calib_data,
					    &sensor_position) != 0) {
			continue;
		}

		robot_pose_vars.sensor_positions[sensor_idx] = sensor_position;
		local_points[valid_count] =
			robot_pose_get_local_sensor_point(sensor_idx);
		world_points[valid_count] = sensor_position;
		valid_mask |= (uint8_t)(1u << sensor_idx);
		valid_count++;
	}

	robot_pose_vars.pose.valid_sensor_count = valid_count;
	robot_pose_vars.pose.valid_sensor_mask = valid_mask;

	if (valid_count < ROBOT_POSE_MIN_VALID_SENSORS) {
		robot_pose_vars.pose.valid = false;
		robot_pose_vars.calibrated_pose.valid = false;
		return -2;
	}

	for (uint8_t i = 0; i < valid_count; i++) {
		local_mean.x += local_points[i].x;
		local_mean.y += local_points[i].y;
		world_mean.x += world_points[i].x;
		world_mean.y += world_points[i].y;
		world_mean.z += world_points[i].z;
	}

	local_mean.x /= valid_count;
	local_mean.y /= valid_count;
	world_mean.x /= valid_count;
	world_mean.y /= valid_count;
	world_mean.z /= valid_count;

	for (uint8_t i = 0; i < valid_count; i++) {
		double lx = local_points[i].x - local_mean.x;
		double ly = local_points[i].y - local_mean.y;
		double wx = world_points[i].x - world_mean.x;
		double wy = world_points[i].y - world_mean.y;

		dot_sum += lx * wx + ly * wy;
		cross_sum += lx * wy - ly * wx;
	}

	if (fabs(dot_sum) < 1e-9 && fabs(cross_sum) < 1e-9) {
		robot_pose_vars.pose.valid = false;
		robot_pose_vars.calibrated_pose.valid = false;
		return -3;
	}

	double heading_rad = atan2(cross_sum, dot_sum);
	double cos_heading = cos(heading_rad);
	double sin_heading = sin(heading_rad);

	robot_pose_vars.pose.position.x =
		world_mean.x -
		(cos_heading * local_mean.x - sin_heading * local_mean.y);
	robot_pose_vars.pose.position.y =
		world_mean.y -
		(sin_heading * local_mean.x + cos_heading * local_mean.y);
	robot_pose_vars.pose.position.z = world_mean.z;
	robot_pose_vars.pose.heading_deg = robot_pose_normalize_heading(
		(float)(heading_rad * 180.0 / M_PI));
	robot_pose_vars.pose.valid = true;
	robot_pose_update_calibrated_pose();

	return 0;
}

const robot_pose_t *robot_pose_get(void)
{
	robot_pose_update();
	return &robot_pose_vars.pose;
}

const lighthouse_point *robot_pose_get_position(void)
{
	return &robot_pose_vars.pose.position;
}

bool robot_pose_get_heading_deg(float *heading_deg)
{
	if (heading_deg == NULL || !robot_pose_vars.pose.valid) {
		return false;
	}

	*heading_deg = robot_pose_vars.pose.heading_deg;
	return true;
}

bool robot_pose_is_valid(void)
{
	return robot_pose_vars.pose.valid;
}

bool robot_pose_set_reference(const lighthouse_point *position, float heading_deg)
{
	if (position == NULL) {
		return false;
	}

	robot_pose_vars.reference_pose.valid = true;
	robot_pose_vars.reference_pose.position = *position;
	robot_pose_vars.reference_pose.heading_deg =
		robot_pose_normalize_heading(heading_deg);
	robot_pose_update_calibrated_pose();

	return true;
}

bool robot_pose_set_reference_from_current(void)
{
	if (!robot_pose_vars.pose.valid) {
		return false;
	}

	return robot_pose_set_reference(&robot_pose_vars.pose.position,
					robot_pose_vars.pose.heading_deg);
}

void robot_pose_clear_reference(void)
{
	robot_pose_vars.reference_pose.valid = false;
	robot_pose_vars.calibrated_pose.valid = false;
}

bool robot_pose_has_reference(void)
{
	return robot_pose_vars.reference_pose.valid;
}

bool robot_pose_get_calibrated(robot_pose_calibrated_t *out_pose)
{
	if (out_pose == NULL || !robot_pose_vars.calibrated_pose.valid) {
		return false;
	}

	*out_pose = robot_pose_vars.calibrated_pose;
	return true;
}

static lighthouse_point robot_pose_get_local_sensor_point(uint8_t sensor_idx)
{
	double angle_deg = 0.0;
	double angle_rad;

	switch (sensor_idx) {
	case 0:
		angle_deg = ROBOT_POSE_SENSOR0_ANGLE_DEG;
		break;
	case 1:
		angle_deg = ROBOT_POSE_SENSOR1_ANGLE_DEG;
		break;
	case 2:
		angle_deg = ROBOT_POSE_SENSOR2_ANGLE_DEG;
		break;
	default:
		break;
	}

	angle_rad = angle_deg * M_PI / 180.0;

	return (lighthouse_point){
		ROBOT_POSE_SENSOR_RADIUS_MM * cos(angle_rad),
		ROBOT_POSE_SENSOR_RADIUS_MM * sin(angle_rad),
		0.0};
}

static float robot_pose_normalize_heading(float heading_deg)
{
	while (heading_deg < 0.0f) {
		heading_deg += 360.0f;
	}
	while (heading_deg >= 360.0f) {
		heading_deg -= 360.0f;
	}
	return heading_deg;
}

static float robot_pose_normalize_relative_heading(float heading_deg)
{
	while (heading_deg >= 180.0f) {
		heading_deg -= 360.0f;
	}
	while (heading_deg < -180.0f) {
		heading_deg += 360.0f;
	}
	return heading_deg;
}

static void robot_pose_update_calibrated_pose(void)
{
	double reference_heading_rad;
	double sin_reference_heading;
	double cos_reference_heading;
	lighthouse_point delta_world;

	if (!robot_pose_vars.pose.valid || !robot_pose_vars.reference_pose.valid) {
		robot_pose_vars.calibrated_pose.valid = false;
		return;
	}

	delta_world.x = robot_pose_vars.pose.position.x -
			robot_pose_vars.reference_pose.position.x;
	delta_world.y = robot_pose_vars.pose.position.y -
			robot_pose_vars.reference_pose.position.y;
	delta_world.z = robot_pose_vars.pose.position.z -
			robot_pose_vars.reference_pose.position.z;

	reference_heading_rad =
		robot_pose_vars.reference_pose.heading_deg * M_PI / 180.0;
	sin_reference_heading = sin(reference_heading_rad);
	cos_reference_heading = cos(reference_heading_rad);

	robot_pose_vars.calibrated_pose.position.x =
		delta_world.x * cos_reference_heading +
		delta_world.y * sin_reference_heading;
	robot_pose_vars.calibrated_pose.position.y =
		-delta_world.x * sin_reference_heading +
		delta_world.y * cos_reference_heading;
	robot_pose_vars.calibrated_pose.position.z = delta_world.z;
	robot_pose_vars.calibrated_pose.heading_deg =
		robot_pose_normalize_relative_heading(
			robot_pose_vars.pose.heading_deg -
			robot_pose_vars.reference_pose.heading_deg);
	robot_pose_vars.calibrated_pose.valid = true;
}
