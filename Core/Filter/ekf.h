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

#define QNORM_EPSILON 	1e-6f

typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;

    float angle;
    float bias;
    float rate;
    float filtered_rate;

    float P[2][2];

} Kalman_t;

float kalman_filter_roll(Kalman_t *Kalman, tAccelFloat* accelf, float newRate, float dt);
float kalman_filter_pitch(Kalman_t *Kalman, tAccelFloat* accelf, float newRate, float dt);


/* functions */
void quaternion_to_euler(tAHRS* ahrs, tBNO055* bno);

void init_matrices();

void lla_to_ecef(tPosition* pos);
void ecef_to_ned(const tECEF_Position* ecef, tNED_Position* ned, float lat_ref, float lon_ref);

void quaternion_to_dcm(tAHRS* ahrs, float R[3][3]);
void transform_accel_to_ned(tAHRS* ahrs);

void ekf_init(tAHRS* ahrs, tBNO055* bno) ;
void ekf_predict(tAHRS* ahrs, tBNO055* bno, float dt);
void ekf_update(tAHRS* ahrs, tBNO055* bno);

void ekf_task(t_gps* gps, tBNO055* bno, tAHRS* ahrs);

void update_position_ned(tAHRS* ahrs);



#endif /* FILTER_EKF_H_ */
