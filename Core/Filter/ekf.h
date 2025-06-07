/*
 * ekf.h
 *
 *  Created on: Jun 1, 2025
 *      Author: selenevrenkaya
 */

#ifndef FILTER_EKF_H_
#define FILTER_EKF_H_

#include "ahrs.h"
#include "bno055.h"
#include "neo_m8n.h"


/* definitions */
#define EARTH_RADIUS 	6378137.0f
#define ECC2 			0.00669437999014f
#define DEG2RAD 		(M_PI / 180.0f)
#define GRAVITY 		9.80665f


/* functions */

void init_matrices();

void lla_to_ecef(tPosition* pos);
void ecef_to_ned(const tECEF_Position* ecef, tNED_Position* ned, float lat_ref, float lon_ref);

void quaternion_to_dcm(tAHRS* ahrs, float R[3][3]);
void transform_accel_to_ned(tAHRS* ahrs);

void ekf_init(tAHRS* ahrs);
void ekf_predict(tAHRS* ahrs, float dt);
void ekf_update(tAHRS* ahrs);

void ekf_task(t_gps* gps, tAHRS* ahrs);

void update_position_ned(tAHRS* ahrs);



#endif /* FILTER_EKF_H_ */
