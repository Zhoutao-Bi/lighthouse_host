#ifndef LIGHTHOUSE_CONFIG_H
#define LIGHTHOUSE_CONFIG_H

/* 2D vs 3D mode switch.
 *
 * 0 = single base station, 2D plane (z = 0). Default.
 * 1 = dual base station, full 3D via skew lines triangulation
 *     (NOT YET IMPLEMENTED — returns -ENOTSUP).
 *
 * To enable 3D once algorithm is ready:
 *   1. Implement lighthouse_get_position_3d() in pos.c
 *   2. Set LIGHTHOUSE_MODE_3D to 1 below
 *   3. Provide calib_data_b in main.c (second base station)
 */
#define LIGHTHOUSE_MODE_3D 0

#endif /* LIGHTHOUSE_CONFIG_H */
