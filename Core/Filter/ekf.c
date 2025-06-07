/*
 * ekfilter.c
 *
 *  Created on: Jun 1, 2025
 *      Author: selenevrenkaya
 */

#include "ekf.h"
#include "ahrs.h"

#include <math.h>
#include"stdbool.h"

static tPosition pos_ref; // Reference position

bool init_check = false;

// EKF internal states and matrices
float X[15];              // State vector
float P[15][15];          // Covariance matrix
float Q[15][15];          // Process noise covariance
float F[15][15];          // State transition Jacobian
float H[6][15];           // Measurement matrix (only GPS)
float R[6][6];            // Measurement noise covariance
float K[15][6];           // Kalman gain
float Y[6];               // Innovation vector



void init_matrices() {
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
            Q[i][j] = (i == j) ? 0.01f : 0.0f;
            F[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 15; j++) {
            H[i][j] = 0.0f;
        }
        H[i][i] = 1.0f;
    }
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            R[i][j] = (i == j) ? 2.0f : 0.0f;
        }
    }
}


void lla_to_ecef(tPosition* pos) {
    float lat = pos->latitude * DEG2RAD;
    float lon = pos->longitude * DEG2RAD;
    float alt = pos->altitude;

    float sin_lat = sinf(lat);
    float cos_lat = cosf(lat);
    float sin_lon = sinf(lon);
    float cos_lon = cosf(lon);

    float denom = sqrtf(1.0f - ECC2 * sin_lat * sin_lat);
    float Rew = EARTH_RADIUS / denom;

    pos->ecef.x = (Rew + alt) * cos_lat * cos_lon;
    pos->ecef.y = (Rew + alt) * cos_lat * sin_lon;
    pos->ecef.z = (Rew * (1.0f - ECC2) + alt) * sin_lat;
}


void ecef_to_ned(const tECEF_Position* ecef, tNED_Position* ned, float lat_ref, float lon_ref) {
    float sin_lat = sinf(lat_ref);
    float cos_lat = cosf(lat_ref);
    float sin_lon = sinf(lon_ref);
    float cos_lon = cosf(lon_ref);

    float dx = ecef->x - pos_ref.ecef.x;
    float dy = ecef->y - pos_ref.ecef.y;
    float dz = ecef->z - pos_ref.ecef.z;

    ned->north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
    ned->east  = -sin_lon * dx + cos_lon * dy;
    ned->down  = -cos_lat * cos_lon * dx - cos_lat * sin_lon * dy - sin_lat * dz;
}


void quaternion_to_dcm(tAHRS* ahrs, float R[3][3]) {
    float q0 = ahrs->telemetry.q0 / 10000.0f;	// 10000.0f ile normalizasyon
    float q1 = ahrs->telemetry.q1 / 10000.0f;
    float q2 = ahrs->telemetry.q2 / 10000.0f;
    float q3 = ahrs->telemetry.q3 / 10000.0f;

    R[0][0] = 1.0f - 2.0f * (q2*q2 + q3*q3);
    R[0][1] = 2.0f * (q1*q2 - q0*q3);
    R[0][2] = 2.0f * (q1*q3 + q0*q2);
    R[1][0] = 2.0f * (q1*q2 + q0*q3);
    R[1][1] = 1.0f - 2.0f * (q1*q1 + q3*q3);
    R[1][2] = 2.0f * (q2*q3 - q0*q1);
    R[2][0] = 2.0f * (q1*q3 - q0*q2);
    R[2][1] = 2.0f * (q2*q3 + q0*q1);
    R[2][2] = 1.0f - 2.0f * (q1*q1 + q2*q2);
}


void transform_accel_to_ned(tAHRS* ahrs) {
    float R[3][3];
    quaternion_to_dcm(ahrs, R);

    float ax = ahrs->telemetry.aX;
    float ay = ahrs->telemetry.aY;
    float az = ahrs->telemetry.aZ;

    ahrs->telemetry.aN_ned = R[0][0]*ax + R[0][1]*ay + R[0][2]*az;
    ahrs->telemetry.aE_ned = R[1][0]*ax + R[1][1]*ay + R[1][2]*az;
    ahrs->telemetry.aD_ned = R[2][0]*ax + R[2][1]*ay + R[2][2]*az;
}


void ekf_init(tAHRS* ahrs) {

    lla_to_ecef(&ahrs->position);
    update_position_ned(ahrs);
    pos_ref = ahrs->position;

    init_matrices();

    X[0] = ahrs->position.ned.north;
    X[1] = ahrs->position.ned.east;
    X[2] = ahrs->position.ned.down;
    X[3] = 0.0f;
    X[4] = 0.0f;
    X[5] = 0.0f;

}


void ekf_predict(tAHRS* ahrs, float dt) {
    for (int i = 0; i < 3; i++) {
        X[i] += X[i + 3] * dt;
    }

    transform_accel_to_ned(ahrs);

    X[3] += ahrs->telemetry.aN_ned * dt;
    X[4] += ahrs->telemetry.aE_ned * dt;
    X[5] += (ahrs->telemetry.aD_ned + GRAVITY) * dt;

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            P[i][j] += Q[i][j] * dt;
        }
    }
}


void ekf_update(tAHRS* ahrs) {

    lla_to_ecef(&ahrs->position);
    update_position_ned(ahrs);

    Y[0] = ahrs->position.ned.north - X[0];
    Y[1] = ahrs->position.ned.east - X[1];
    Y[2] = ahrs->position.ned.down - X[2];
    Y[3] = ahrs->velocity.ned.north - X[3];
    Y[4] = ahrs->velocity.ned.east - X[4];
    Y[5] = ahrs->velocity.ned.down - X[5];

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 6; j++) {
            K[i][j] = P[i][j] / (R[j][j] + 1e-6f);
        }
    }
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 6; j++) {
            X[i] += K[i][j] * Y[j];
        }
    }
    // P = (I - KH) * P
    float I_KH[15][15];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            float kh = 0.0f;
            for (int k = 0; k < 6; k++) {
                kh += K[i][k] * H[k][j];
            }
            I_KH[i][j] = (i == j ? 1.0f : 0.0f) - kh;
        }
    }
    float newP[15][15];
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            newP[i][j] = 0.0f;
            for (int k = 0; k < 15; k++) {
                newP[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            P[i][j] = newP[i][j];
        }
    }
}

void ekf_task(t_gps* gps, tAHRS* ahrs){

	if((gps->receive_check == true) && (init_check == false)){
		if((gps->gga.location.latitude > 10.00) && (gps->gga.location.longitude > 10.00)){

			ekf_init(ahrs);
			init_check = true;
		}
	}

	ekf_predict(ahrs, 0.01f);
	ekf_update(ahrs);


}


void update_position_ned(tAHRS* ahrs) {
    float lat_rad = ahrs->position.latitude * DEG2RAD;
    float lon_rad = ahrs->position.longitude * DEG2RAD;

    ecef_to_ned(&ahrs->position.ecef, &ahrs->position.ned, lat_rad, lon_rad);

}

