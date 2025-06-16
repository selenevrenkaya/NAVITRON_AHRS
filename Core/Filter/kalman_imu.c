/*
 * kalman_imu.c
 *
 *  Created on: Jun 14, 2025
 *      Author: selenevrenkaya
 */

#include "ekf.h"

float Q_angle;
float Q_bias;
float R_measure;

float angle;
float bias;
float rate;

Kalman_t kalman_roll = {	.Q_angle 	= 0.1f,	// 0.001f
							.Q_bias		= 0.003f,
							.R_measure	= 0.01f,	//0.03f
							.angle		= 0.0f,
							.bias		= 0.0f,
							.P[0][0]	= 0.0f,
							.P[0][1]	= 0.0f,
							.P[1][0]	= 0.0f,
							.P[1][1]	= 0.0f,			};

Kalman_t kalman_pitch = {	.Q_angle 	= 0.1f,	// 0.001f
							.Q_bias		= 0.003f,
							.R_measure	= 0.01f,	//0.03f
							.angle		= 0.0f,
							.bias		= 0.0f,
							.P[0][0]	= 0.0f,
							.P[0][1]	= 0.0f,
							.P[1][0]	= 0.0f,
							.P[1][1]	= 0.0f,			};


float kalman_filter_roll(Kalman_t *Kalman, tAccelFloat* accelf, float newRate, float dt){

	float alpha = 0.8;
	Kalman->filtered_rate = alpha * Kalman->filtered_rate + (1 - alpha) * newRate;
	float rate = Kalman->filtered_rate - Kalman->bias;

	//float rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

	float S = Kalman->P[0][0] + Kalman->R_measure;

	float K[2];
	K[0] = Kalman->P[0][0] / S;
	K[1] = Kalman->P[1][0] / S;

	float roll_acc = atan2(accelf->X, sqrt(accelf->Y * accelf->Y + accelf->Z * accelf->Z)) * 180 / M_PI;

	float y = roll_acc - Kalman->angle;
	Kalman->angle += K[0] * y;
	Kalman->bias += K[1] * y;

	float P00_temp = Kalman->P[0][0];
	float P01_temp = Kalman->P[0][1];

	Kalman->P[0][0] -= K[0] * P00_temp;
	Kalman->P[0][1] -= K[0] * P01_temp;
	Kalman->P[1][0] -= K[1] * P00_temp;
	Kalman->P[1][1] -= K[1] * P01_temp;


	return Kalman->angle;
}


float kalman_filter_pitch(Kalman_t *Kalman, tAccelFloat* accelf, float newRate, float dt) {

    float rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;

    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    // calculates pitch angle from accel value
    float pitch_acc = atan2(-accelf->Y, sqrt(accelf->X * accelf->X + accelf->Z * accelf->Z)) * 180 / M_PI;

    float y = pitch_acc - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}



