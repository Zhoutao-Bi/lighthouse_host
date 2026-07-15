/*
 * Copyright (c) Regents of the University of California.
 * All rights reserved.
 *
 * Ported to Zephyr / NCS v3.4.0 from atom-robot firmware.
 * Original author: Cheng Wang <cwang199@connect.hkust-gz.edu.cn>, December 2023.
 * Reference: Open source project https://github.com/HiveTracker/firmware
 *            QingFeng BBS http://www.qfv8.com/
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "pos.h"
#include "ppi.h"
#include "../lighthouse_config.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
	float A_X_theta_beta[POS_MAX_SENSORS];
	float A_Y_theta_alpha[POS_MAX_SENSORS];
	uint8_t data_xy[POS_MAX_SENSORS][6];
	lighthouse_result base_station;
	lighthouse_point robot_pos[POS_MAX_SENSORS];

	float x_estimate[POS_MAX_SENSORS];
	float x_error_estimate[POS_MAX_SENSORS];
	float y_estimate[POS_MAX_SENSORS];
	float y_error_estimate[POS_MAX_SENSORS];

	uint32_t t_d_start[POS_MAX_SENSORS];
	uint8_t flag_light_type[POS_MAX_SENSORS];
	uint8_t flag_station[POS_MAX_SENSORS];
	uint8_t flag_A_station[POS_MAX_SENSORS];
	uint32_t loca_duration_ticks[POS_MAX_SENSORS];
	uint8_t loca_x[POS_MAX_SENSORS];
	uint32_t A_X_ticks[POS_MAX_SENSORS];
	uint32_t A_Y_ticks[POS_MAX_SENSORS];
	uint8_t data[POS_MAX_SENSORS][6];

	int i_test[POS_MAX_SENSORS];
	unsigned long int test[POS_MAX_SENSORS][500];
	int i_test2[POS_MAX_SENSORS];
	uint32_t test2[POS_MAX_SENSORS][500];
	uint32_t test3[POS_MAX_SENSORS][500];
} pos_vars_t;

typedef struct {
	double m[3][3];
} Mat3;

#define TYPE_SYNC      0u
#define TYPE_SWEEP     1u
#define TYPE_SKIP_SYNC 2u
#define POS_TEST_BUF_SIZE 500u
#define POS_X_Q  0.01f
#define POS_X_R  0.1f
#define POS_Y_Q  0.001f
#define POS_Y_R  0.1f

static float kalman_filter(float measurement, float *estimate,
			   float *error_estimate, float q, float r);
static double clamp_val(double v, double min, double max);
static lighthouse_point vec_sub(lighthouse_point a, lighthouse_point b);
static lighthouse_point vec_add(lighthouse_point a, lighthouse_point b);
static lighthouse_point vec_scale(lighthouse_point a, double s);
static double vec_dot(lighthouse_point a, lighthouse_point b);
static double vec_norm(lighthouse_point a);
static lighthouse_point vec_cross(lighthouse_point a, lighthouse_point b);
static lighthouse_point vec_normalize(lighthouse_point a);
static Mat3 mat_mul(Mat3 A, Mat3 B);
static Mat3 mat_transpose(Mat3 A);
static double mat_det(Mat3 A);
static int mat_inverse(Mat3 A, Mat3 *inv);
static void svd_3x3_robust(Mat3 H, Mat3 *U, Mat3 *V);
static LH_Euler matrix_to_euler(Mat3 R);

static pos_vars_t pos_vars;

static float kalman_filter(float measurement, float *estimate,
			   float *error_estimate, float q, float r)
{
	float predict_estimate = *estimate;
	float predict_error = *error_estimate + q;

	float kalman_gain = predict_error / (predict_error + r);
	*estimate = predict_estimate +
		    kalman_gain * (measurement - predict_estimate);
	*error_estimate = (1 - kalman_gain) * predict_error;

	return *estimate;
}

void pos_process_light_signal(uint8_t sensor_idx, uint32_t t_start,
			      uint32_t t_end, uint32_t duration)
{
	if (sensor_idx >= POS_MAX_SENSORS) {
		return;
	}

	uint32_t t_opt_pulse_us = duration / 16;

	pos_vars.test[sensor_idx][pos_vars.i_test[sensor_idx]] = t_opt_pulse_us;
	pos_vars.i_test[sensor_idx] = (pos_vars.i_test[sensor_idx] + 1) %
				      POS_TEST_BUF_SIZE;

	if (duration < 800) {
		pos_vars.flag_light_type[sensor_idx] = TYPE_SWEEP;
	} else if (duration < 1584) {
		pos_vars.flag_light_type[sensor_idx] = TYPE_SYNC;

		pos_vars.test2[sensor_idx][pos_vars.i_test2[sensor_idx]] = t_start;
		if (pos_vars.i_test2[sensor_idx] > 0) {
			pos_vars.test3[sensor_idx][pos_vars.i_test2[sensor_idx] - 1] =
				(pos_vars.test2[sensor_idx][pos_vars.i_test2[sensor_idx]] -
				 pos_vars.test2[sensor_idx][pos_vars.i_test2[sensor_idx] - 1]) /
				16;
		}
		pos_vars.i_test2[sensor_idx] = (pos_vars.i_test2[sensor_idx] + 1) %
					       POS_TEST_BUF_SIZE;
	} else {
		pos_vars.flag_light_type[sensor_idx] = TYPE_SKIP_SYNC;
	}

	switch (pos_vars.flag_light_type[sensor_idx]) {
	case TYPE_SYNC:
		pos_vars.t_d_start[sensor_idx] = t_start;
		if (pos_vars.flag_station[sensor_idx] >= 1) {
			pos_vars.loca_x[sensor_idx] =
				((t_opt_pulse_us / 10) % 2 == 0) ? 1 : 0;
		}
		if (pos_vars.flag_station[sensor_idx] == 1) {
			pos_vars.flag_A_station[sensor_idx] = 1;
		}
		if (pos_vars.flag_station[sensor_idx] == 2) {
			pos_vars.flag_A_station[sensor_idx] = 2;
		}
		break;

	case TYPE_SWEEP:
		pos_vars.flag_station[sensor_idx] = 1;
		pos_vars.loca_duration_ticks[sensor_idx] =
			t_start - pos_vars.t_d_start[sensor_idx];

		if (pos_vars.flag_A_station[sensor_idx] == 1) {
			if (pos_vars.loca_x[sensor_idx] == 1) {
				pos_vars.A_Y_ticks[sensor_idx] =
					pos_vars.loca_duration_ticks[sensor_idx];
				float A_Y_theta_raw = (float)pos_vars.A_Y_ticks[sensor_idx] *
						      360.0f / 8.33f / 2.0f / 16000.0f;
				pos_vars.A_Y_theta_alpha[sensor_idx] = kalman_filter(
					A_Y_theta_raw, &pos_vars.y_estimate[sensor_idx],
					&pos_vars.y_error_estimate[sensor_idx], POS_Y_Q, POS_Y_R);

				pos_vars.data[sensor_idx][3] =
					(pos_vars.A_Y_ticks[sensor_idx] >> 16) & 0xFF;
				pos_vars.data[sensor_idx][4] =
					(pos_vars.A_Y_ticks[sensor_idx] >> 8) & 0xFF;
				pos_vars.data[sensor_idx][5] =
					pos_vars.A_Y_ticks[sensor_idx] & 0xFF;
			} else {
				pos_vars.A_X_ticks[sensor_idx] =
					pos_vars.loca_duration_ticks[sensor_idx];
				float A_X_theta_raw = (float)pos_vars.A_X_ticks[sensor_idx] *
						      360.0f / 8.33f / 2.0f / 16000.0f;
				pos_vars.A_X_theta_beta[sensor_idx] = kalman_filter(
					A_X_theta_raw, &pos_vars.x_estimate[sensor_idx],
					&pos_vars.x_error_estimate[sensor_idx], POS_X_Q, POS_X_R);

				pos_vars.data[sensor_idx][0] =
					(pos_vars.A_X_ticks[sensor_idx] >> 16) & 0xFF;
				pos_vars.data[sensor_idx][1] =
					(pos_vars.A_X_ticks[sensor_idx] >> 8) & 0xFF;
				pos_vars.data[sensor_idx][2] =
					pos_vars.A_X_ticks[sensor_idx] & 0xFF;
			}
			for (int i = 0; i < 6; i++) {
				pos_vars.data_xy[sensor_idx][i] = pos_vars.data[sensor_idx][i];
			}
			pos_vars.flag_A_station[sensor_idx] = 0;
		}
		break;

	default:
		break;
	}
}

void pos_init(void)
{
	memset(&pos_vars, 0, sizeof(pos_vars_t));

	for (uint8_t s = 0; s < POS_MAX_SENSORS; s++) {
		pos_vars.x_estimate[s] = 0.0f;
		pos_vars.x_error_estimate[s] = 1.0f;
		pos_vars.y_estimate[s] = 0.0f;
		pos_vars.y_error_estimate[s] = 1.0f;

		pos_vars.t_d_start[s] = 0;
		pos_vars.flag_light_type[s] = 0;
		pos_vars.flag_station[s] = 0;
		pos_vars.flag_A_station[s] = 0;
		pos_vars.loca_duration_ticks[s] = 0;
		pos_vars.loca_x[s] = 0;
		pos_vars.A_X_ticks[s] = 0;
		pos_vars.A_Y_ticks[s] = 0;
	}

	ppi_set_light_signal_ex_callback(pos_process_light_signal);
}

static double clamp_val(double v, double min, double max)
{
	if (v < min) {
		return min;
	}
	if (v > max) {
		return max;
	}
	return v;
}

static lighthouse_point vec_sub(lighthouse_point a, lighthouse_point b)
{
	return (lighthouse_point){a.x - b.x, a.y - b.y, a.z - b.z};
}

static lighthouse_point vec_add(lighthouse_point a, lighthouse_point b)
{
	return (lighthouse_point){a.x + b.x, a.y + b.y, a.z + b.z};
}

static lighthouse_point vec_scale(lighthouse_point a, double s)
{
	return (lighthouse_point){a.x * s, a.y * s, a.z * s};
}

static double vec_dot(lighthouse_point a, lighthouse_point b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static double vec_norm(lighthouse_point a)
{
	return sqrt(vec_dot(a, a));
}

static lighthouse_point vec_cross(lighthouse_point a, lighthouse_point b)
{
	return (lighthouse_point){a.y * b.z - a.z * b.y,
				  a.z * b.x - a.x * b.z,
				  a.x * b.y - a.y * b.x};
}

static lighthouse_point vec_normalize(lighthouse_point a)
{
	double n = vec_norm(a);
	if (n < 1e-9) {
		return a;
	}
	return vec_scale(a, 1.0 / n);
}

static Mat3 mat_mul(Mat3 A, Mat3 B)
{
	Mat3 C = {{{0}}};
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			for (int k = 0; k < 3; k++) {
				C.m[i][j] += A.m[i][k] * B.m[k][j];
			}
		}
	}
	return C;
}

static Mat3 mat_transpose(Mat3 A)
{
	Mat3 T;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			T.m[i][j] = A.m[j][i];
		}
	}
	return T;
}

static double mat_det(Mat3 A)
{
	return A.m[0][0] * (A.m[1][1] * A.m[2][2] - A.m[1][2] * A.m[2][1]) -
	       A.m[0][1] * (A.m[1][0] * A.m[2][2] - A.m[1][2] * A.m[2][0]) +
	       A.m[0][2] * (A.m[1][0] * A.m[2][1] - A.m[1][1] * A.m[2][0]);
}

static int mat_inverse(Mat3 A, Mat3 *inv)
{
	double det = mat_det(A);
	if (fabs(det) < 1e-9) {
		return 0;
	}
	double invDet = 1.0 / det;
	inv->m[0][0] = (A.m[1][1] * A.m[2][2] - A.m[1][2] * A.m[2][1]) * invDet;
	inv->m[0][1] = (A.m[0][2] * A.m[2][1] - A.m[0][1] * A.m[2][2]) * invDet;
	inv->m[0][2] = (A.m[0][1] * A.m[1][2] - A.m[0][2] * A.m[1][1]) * invDet;
	inv->m[1][0] = (A.m[1][2] * A.m[2][0] - A.m[1][0] * A.m[2][2]) * invDet;
	inv->m[1][1] = (A.m[0][0] * A.m[2][2] - A.m[0][2] * A.m[2][0]) * invDet;
	inv->m[1][2] = (A.m[0][2] * A.m[1][0] - A.m[0][0] * A.m[1][2]) * invDet;
	inv->m[2][0] = (A.m[1][0] * A.m[2][1] - A.m[1][1] * A.m[2][0]) * invDet;
	inv->m[2][1] = (A.m[0][1] * A.m[2][0] - A.m[0][0] * A.m[2][1]) * invDet;
	inv->m[2][2] = (A.m[0][0] * A.m[1][1] - A.m[0][1] * A.m[1][0]) * invDet;
	return 1;
}

static void svd_3x3_robust(Mat3 H, Mat3 *U, Mat3 *V)
{
	Mat3 A = H;
	Mat3 AT = mat_transpose(A);
	Mat3 ATA = mat_mul(AT, A);

	Mat3 V_curr = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

	for (int iter = 0; iter < 50; iter++) {
		double max_off = 0.0;
		int p = 0, q = 1;
		for (int i = 0; i < 3; i++) {
			for (int j = i + 1; j < 3; j++) {
				if (fabs(ATA.m[i][j]) > max_off) {
					max_off = fabs(ATA.m[i][j]);
					p = i;
					q = j;
				}
			}
		}
		if (max_off < 1e-12) {
			break;
		}

		double theta = 0.5 * atan2(2 * ATA.m[p][q], ATA.m[p][p] - ATA.m[q][q]);
		double c = cos(theta), s = sin(theta);

		Mat3 J = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
		J.m[p][p] = c;
		J.m[p][q] = -s;
		J.m[q][p] = s;
		J.m[q][q] = c;

		V_curr = mat_mul(V_curr, J);
		Mat3 temp = mat_mul(mat_transpose(J), ATA);
		ATA = mat_mul(temp, J);
	}
	*V = V_curr;

	double sigma[3];
	for (int i = 0; i < 3; i++) {
		sigma[i] = sqrt(fabs(ATA.m[i][i]));
	}

	Mat3 U_curr = {{{0}}};
	int valid[3] = {0, 0, 0};
	int valid_cnt = 0;

	for (int j = 0; j < 3; j++) {
		if (sigma[j] > 1e-5) {
			lighthouse_point vj = {V_curr.m[0][j], V_curr.m[1][j],
					       V_curr.m[2][j]};
			lighthouse_point Hvj;
			Hvj.x = A.m[0][0] * vj.x + A.m[0][1] * vj.y + A.m[0][2] * vj.z;
			Hvj.y = A.m[1][0] * vj.x + A.m[1][1] * vj.y + A.m[1][2] * vj.z;
			Hvj.z = A.m[2][0] * vj.x + A.m[2][1] * vj.y + A.m[2][2] * vj.z;

			U_curr.m[0][j] = Hvj.x / sigma[j];
			U_curr.m[1][j] = Hvj.y / sigma[j];
			U_curr.m[2][j] = Hvj.z / sigma[j];
			valid[j] = 1;
			valid_cnt++;
		}
	}

	if (valid_cnt == 2) {
		int idx_miss = -1, idx_a = -1, idx_b = -1;
		for (int j = 0; j < 3; j++) {
			if (!valid[j]) {
				idx_miss = j;
			} else if (idx_a == -1) {
				idx_a = j;
			} else {
				idx_b = j;
			}
		}
		lighthouse_point ua = {U_curr.m[0][idx_a], U_curr.m[1][idx_a],
				       U_curr.m[2][idx_a]};
		lighthouse_point ub = {U_curr.m[0][idx_b], U_curr.m[1][idx_b],
				       U_curr.m[2][idx_b]};
		lighthouse_point u_miss = vec_cross(ua, ub);
		U_curr.m[0][idx_miss] = u_miss.x;
		U_curr.m[1][idx_miss] = u_miss.y;
		U_curr.m[2][idx_miss] = u_miss.z;
	}
	*U = U_curr;
}

static LH_Euler matrix_to_euler(Mat3 R)
{
	LH_Euler ea;
	double r11 = R.m[0][0], r21 = R.m[1][0], r31 = R.m[2][0];
	double r32 = R.m[2][1], r33 = R.m[2][2];

	double pitch_rad = asin(clamp_val(-r31, -1.0, 1.0));
	double yaw_rad = atan2(r21, r11);
	double roll_rad = atan2(r32, r33);

	ea.pitch = pitch_rad * 180.0 / M_PI;
	ea.yaw = yaw_rad * 180.0 / M_PI;
	ea.roll = roll_rad * 180.0 / M_PI;
	return ea;
}

int lighthouse_calibrate(const lighthouse_point world_points[],
			 const lighthouse_angles world_angles[],
			 lighthouse_result *out_result)
{
	if (!out_result) {
		return -1;
	}
	memset(out_result, 0, sizeof(lighthouse_result));

	lighthouse_point rays_B[3];
	for (int i = 0; i < 3; i++) {
		double a = world_angles[i].alpha_deg * M_PI / 180.0;
		double b = world_angles[i].beta_deg * M_PI / 180.0;
		if (fabs(fabs(a) - M_PI / 2) < 1e-4) {
			a = (a > 0) ? 1.5707 : -1.5707;
		}
		if (fabs(fabs(b) - M_PI / 2) < 1e-4) {
			b = (b > 0) ? 1.5707 : -1.5707;
		}

		double x = 1.0;
		double y = -1.0 / tan(a);
		double z = -1.0 / tan(b);
		rays_B[i] = vec_normalize((lighthouse_point){x, y, z});
	}

	double dists[3];
	double max_len = 0;
	int p1 = 0, p2 = 1;
	for (int i = 0; i < 3; i++) {
		for (int j = i + 1; j < 3; j++) {
			double len = vec_norm(vec_sub(world_points[i], world_points[j]));
			if (len > max_len) {
				max_len = len;
				p1 = i;
				p2 = j;
			}
		}
	}
	double angle_between =
		acos(clamp_val(vec_dot(rays_B[p1], rays_B[p2]), -1.0, 1.0));
	double d_guess = max_len / (angle_between + 1e-6);
	if (d_guess < 100.0) {
		d_guess = 100.0;
	}
	dists[0] = dists[1] = dists[2] = d_guess;

	double real_lens[3];
	real_lens[0] = vec_norm(vec_sub(world_points[0], world_points[1]));
	real_lens[1] = vec_norm(vec_sub(world_points[0], world_points[2]));
	real_lens[2] = vec_norm(vec_sub(world_points[1], world_points[2]));

	for (int iter = 0; iter < 50; iter++) {
		lighthouse_point p[3];
		for (int i = 0; i < 3; i++) {
			p[i] = vec_scale(rays_B[i], dists[i]);
		}

		double l[3];
		l[0] = vec_norm(vec_sub(p[0], p[1]));
		l[1] = vec_norm(vec_sub(p[0], p[2]));
		l[2] = vec_norm(vec_sub(p[1], p[2]));

		double r[3] = {l[0] - real_lens[0], l[1] - real_lens[1],
			       l[2] - real_lens[2]};
		if (fabs(r[0]) + fabs(r[1]) + fabs(r[2]) < 1e-5) {
			break;
		}

		double c01 = vec_dot(rays_B[0], rays_B[1]);
		double c02 = vec_dot(rays_B[0], rays_B[2]);
		double c12 = vec_dot(rays_B[1], rays_B[2]);

		Mat3 J;
		J.m[0][0] = (dists[0] - dists[1] * c01) / l[0];
		J.m[0][1] = (dists[1] - dists[0] * c01) / l[0];
		J.m[0][2] = 0;
		J.m[1][0] = (dists[0] - dists[2] * c02) / l[1];
		J.m[1][1] = 0;
		J.m[1][2] = (dists[2] - dists[0] * c02) / l[1];
		J.m[2][0] = 0;
		J.m[2][1] = (dists[1] - dists[2] * c12) / l[2];
		J.m[2][2] = (dists[2] - dists[1] * c12) / l[2];

		Mat3 J_inv;
		if (!mat_inverse(J, &J_inv)) {
			break;
		}

		double delta[3] = {0, 0, 0};
		for (int i = 0; i < 3; i++) {
			for (int k = 0; k < 3; k++) {
				delta[i] -= J_inv.m[i][k] * r[k];
			}
		}

		for (int i = 0; i < 3; i++) {
			dists[i] += delta[i];
			if (dists[i] < 10.0) {
				dists[i] = 10.0;
			}
		}
	}

	lighthouse_point pts_B[3];
	for (int i = 0; i < 3; i++) {
		pts_B[i] = vec_scale(rays_B[i], dists[i]);
	}

	lighthouse_point cA = {0, 0, 0}, cB = {0, 0, 0};
	for (int i = 0; i < 3; i++) {
		cA = vec_add(cA, world_points[i]);
		cB = vec_add(cB, pts_B[i]);
	}
	cA = vec_scale(cA, 1.0 / 3.0);
	cB = vec_scale(cB, 1.0 / 3.0);

	Mat3 H = {{{0}}};
	for (int i = 0; i < 3; i++) {
		lighthouse_point qa = vec_sub(world_points[i], cA);
		lighthouse_point qb = vec_sub(pts_B[i], cB);
		for (int row = 0; row < 3; row++) {
			for (int col = 0; col < 3; col++) {
				H.m[row][col] += ((double *)&qb)[row] * ((double *)&qa)[col];
			}
		}
	}

	Mat3 U, V;
	svd_3x3_robust(H, &U, &V);

	Mat3 R = mat_mul(V, mat_transpose(U));

	if (mat_det(R) < 0) {
		V.m[0][2] *= -1;
		V.m[1][2] *= -1;
		V.m[2][2] *= -1;
		R = mat_mul(V, mat_transpose(U));
	}

	lighthouse_point RcB;
	for (int i = 0; i < 3; i++) {
		((double *)&RcB)[i] =
			R.m[i][0] * cB.x + R.m[i][1] * cB.y + R.m[i][2] * cB.z;
	}

	out_result->valid = 1;
	out_result->position = vec_sub(cA, RcB);
	out_result->attitude = matrix_to_euler(R);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out_result->R[i][j] = R.m[i][j];
		}
	}

	pos_vars.base_station = *out_result;

	return 0;
}

int lighthouse_get_position_simple(const lighthouse_result *calib_data,
				   double alpha_deg, double beta_deg,
				   lighthouse_point *out_pos)
{
	if (!calib_data || !calib_data->valid || !out_pos) {
		return -2;
	}

	double alpha_rad = alpha_deg * M_PI / 180.0;
	double beta_rad = beta_deg * M_PI / 180.0;

	if (fabs(cos(alpha_rad)) < 1e-6) {
		alpha_rad += 1e-5;
	}
	if (fabs(cos(beta_rad)) < 1e-6) {
		beta_rad += 1e-5;
	}

	double x_b = 1.0;
	double y_b = -1.0 / tan(alpha_rad);
	double z_b = -1.0 / tan(beta_rad);

	lighthouse_point ray_local = {x_b, y_b, z_b};
	ray_local = vec_normalize(ray_local);

	lighthouse_point ray_world;
	ray_world.x = calib_data->R[0][0] * ray_local.x +
		      calib_data->R[0][1] * ray_local.y +
		      calib_data->R[0][2] * ray_local.z;
	ray_world.y = calib_data->R[1][0] * ray_local.x +
		      calib_data->R[1][1] * ray_local.y +
		      calib_data->R[1][2] * ray_local.z;
	ray_world.z = calib_data->R[2][0] * ray_local.x +
		      calib_data->R[2][1] * ray_local.y +
		      calib_data->R[2][2] * ray_local.z;

	double fixed_z = 0.0;

	if (fabs(ray_world.z) < 1e-5) {
		return -1;
	}

	double origin_z = calib_data->position.z;
	double lambda = (fixed_z - origin_z) / ray_world.z;

	out_pos->x = calib_data->position.x + lambda * ray_world.x;
	out_pos->y = calib_data->position.y + lambda * ray_world.y;
	out_pos->z = fixed_z;

	return 0;
}

const lighthouse_result *pos_get_base_station(void)
{
	return &pos_vars.base_station;
}

float pos_get_A_X_theta_beta(uint8_t sensor_idx)
{
	if (sensor_idx >= POS_MAX_SENSORS) {
		return 0.0f;
	}
	return pos_vars.A_X_theta_beta[sensor_idx];
}

float pos_get_A_Y_theta_alpha(uint8_t sensor_idx)
{
	if (sensor_idx >= POS_MAX_SENSORS) {
		return 0.0f;
	}
	return pos_vars.A_Y_theta_alpha[sensor_idx];
}

const uint8_t *pos_get_data_xy(uint8_t sensor_idx)
{
	if (sensor_idx >= POS_MAX_SENSORS) {
		return NULL;
	}
	return pos_vars.data_xy[sensor_idx];
}

const lighthouse_point *pos_get_robot_pos(uint8_t sensor_idx)
{
	if (sensor_idx >= POS_MAX_SENSORS) {
		return NULL;
	}
	return &pos_vars.robot_pos[sensor_idx];
}

int pos_get_sensor_position(uint8_t sensor_idx,
			    const lighthouse_result *calib_data,
			    lighthouse_point *out_pos)
{
	if (sensor_idx >= POS_MAX_SENSORS || out_pos == NULL) {
		return -1;
	}

	int ret;
#if LIGHTHOUSE_MODE_3D
	ARG_UNUSED(calib_data);
	ret = -ENOTSUP;
#else
	ret = lighthouse_get_position_simple(
		calib_data, pos_vars.A_Y_theta_alpha[sensor_idx],
		pos_vars.A_X_theta_beta[sensor_idx], out_pos);
#endif
	if (ret == 0) {
		pos_vars.robot_pos[sensor_idx] = *out_pos;
	}
	return ret;
}

int lighthouse_get_position_3d(const lighthouse_result *calib_a,
			      const lighthouse_result *calib_b,
			      double alpha_a_deg, double beta_a_deg,
			      double alpha_b_deg, double beta_b_deg,
			      lighthouse_point *out_pos)
{
	ARG_UNUSED(calib_a);
	ARG_UNUSED(calib_b);
	ARG_UNUSED(alpha_a_deg);
	ARG_UNUSED(beta_a_deg);
	ARG_UNUSED(alpha_b_deg);
	ARG_UNUSED(beta_b_deg);
	ARG_UNUSED(out_pos);
	return -ENOTSUP;
}

uint8_t pos_get_sensor_count(void)
{
	return POS_MAX_SENSORS;
}
