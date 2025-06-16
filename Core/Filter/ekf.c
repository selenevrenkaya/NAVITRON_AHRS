/*
 * ekfilter.c
 *
 *  Created on: Jun 1, 2025
 *      Author: selenevrenkaya
 */

#include "ekf.h"
#include "ahrs.h"
#include "bno055.h"

#include <math.h>
#include"stdbool.h"

#define alpha	(0.02f)

static tPosition pos_ref; // Reference position
static void normalize_quat(tBNO055* bno);
static void euler2quaternion(tAHRS* ahrs);


bool init_check = false;
tBNO055 temp_bno = {0};

extern Kalman_t kalman_roll;
extern Kalman_t kalman_pitch;


// EKF internal states and matrices
float X[15];              // State vector
float P[15][15];          // Covariance matrix
float Q[15][15];          // Process noise covariance
float F[15][15];          // State transition Jacobian
float H[6][15];           // Measurement matrix (only GPS)
float R[6][6];            // Measurement noise covariance
float K[15][6];           // Kalman gain
float Y[6];               // Innovation vector
float I[15][15];          // Identity matrix


// Normalize quaternion
static void normalize_quat(tBNO055* bno) {
    float norm = sqrtf((bno->quaternion.W * bno->quaternion.W) + (bno->quaternion.X * bno->quaternion.X)
    					+ (bno->quaternion.Y * bno->quaternion.Y) + (bno->quaternion.Z * bno->quaternion.Z));

    if (norm > QNORM_EPSILON) {
    	bno->quaternion.W /= norm;
    	bno->quaternion.X /= norm;
    	bno->quaternion.Y /= norm;
    	bno->quaternion.Z /= norm;
    }

}


void quaternion_to_euler(tAHRS* ahrs, tBNO055* bno) {
    float q0 = bno->quaternion.W;
    float q1 = bno->quaternion.X;
    float q2 = bno->quaternion.Y;
    float q3 = bno->quaternion.Z;

    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    ahrs->telemetry.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1.0f)
        ahrs->telemetry.pitch = copysignf(90.0f, sinp); // Out of range
    else
        ahrs->telemetry.pitch = asinf(sinp) * 180.0f / M_PI;

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    ahrs->telemetry.head = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
}


static void euler2quaternion(tAHRS* ahrs){
    /* degree to radian */
	float deg2rad = M_PI / 180.0;

	float yaw_rad = ahrs->telemetry.head * deg2rad;
	float pitch_rad = ahrs->telemetry.pitch * deg2rad;
	float roll_rad = ahrs->telemetry.roll * deg2rad;


	/* Yari acilarin trigonometrik fonksiyonlari */
    float cy = cos(yaw_rad * 0.5);    	/* yaw (Z) */
    float sy = sin(yaw_rad * 0.5);
    float cp = cos(pitch_rad * 0.5);	/* pitch (Y) */
    float sp = sin(pitch_rad * 0.5);
    float cr = cos(roll_rad * 0.5);   	/* roll (X) */
    float sr = sin(roll_rad * 0.5);

    ahrs->telemetry.q0 = cr * cp * cy + sr * sp * sy;
    ahrs->telemetry.q1 = sr * cp * cy - cr * sp * sy;
    ahrs->telemetry.q2 = cr * sp * cy + sr * cp * sy;
    ahrs->telemetry.q3 = cr * cp * sy - sr * sp * cy;

	return ;
}


void init_matrices() {
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;
            Q[i][j] = (i == j) ? 0.01f : 0.0f;
            F[i][j] = (i == j) ? 1.0f : 0.0f;
            I[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 15; j++) H[i][j] = 0.0f;
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


void update_position_ned(tAHRS* ahrs) {
    float lat_rad = ahrs->position.latitude * DEG2RAD;
    float lon_rad = ahrs->position.longitude * DEG2RAD;

    ecef_to_ned(&ahrs->position.ecef, &ahrs->position.ned, lat_rad, lon_rad);

}



void ekf_init(tAHRS* ahrs, tBNO055* bno) {

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
    X[6] = bno->eulerf.R;	// roll
    X[7] = bno->eulerf.P;	// pitch
    X[8] = bno->eulerf.H;	// head
    X[9] = 0.0f;  // Acc bias X (tahmini)
    X[10] = 0.0f; // Acc bias Y
    X[11] = 0.0f; // Acc bias Z
    X[12] = 0.0f; // Gyro bias X
    X[13] = 0.0f; // Gyro bias Y
    X[14] = 0.0f; // Gyro bias Z

}


void ekf_predict(tAHRS* ahrs, tBNO055* bno, float dt) {
    for (int i = 0; i < 3; i++) {
        X[i] += X[i + 3] * dt;
    }

    transform_accel_to_ned(ahrs);

    X[3] += ahrs->telemetry.aN_ned * dt;
    X[4] += ahrs->telemetry.aE_ned * dt;
    X[5] += (ahrs->telemetry.aD_ned + GRAVITY) * dt;

    // Euler açıları için basit integrasyon
    X[6] += (bno->gyro.X - X[12]) * dt;
    X[7] += (bno->gyro.Y - X[13]) * dt;
    X[8] += (bno->gyro.Z - X[14]) * dt;

    // P = F * P * F^T + Q
    float FP[15][15] = {0};
    float FTP[15][15] = {0};

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            for (int k = 0; k < 15; k++) {
                FP[i][j] += F[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            FTP[i][j] = 0;
            for (int k = 0; k < 15; k++) {
                FTP[i][j] += FP[i][k] * F[j][k];
            }
            P[i][j] = FTP[i][j] + Q[i][j];
        }
    }

    // Çeyrek adımlı integrasyon (small angle approx)
    float w_temp = 1.0f;
    float x_temp = 0.5f * bno->quaternion.X * dt;
    float y_temp = 0.5f * bno->quaternion.Y * dt;
    float z_temp = 0.5f * bno->quaternion.Z * dt;

    // Mevcut durumla çarp (q = q * dq)
    temp_bno.quaternion.W = bno->quaternion.W * w_temp - bno->quaternion.X * x_temp - bno->quaternion.Y * y_temp - bno->quaternion.Z * z_temp;
    temp_bno.quaternion.X = bno->quaternion.W * w_temp + bno->quaternion.X * x_temp + bno->quaternion.Y * y_temp - bno->quaternion.Z * z_temp;
    temp_bno.quaternion.Y = bno->quaternion.W * w_temp - bno->quaternion.X * x_temp + bno->quaternion.Y * y_temp + bno->quaternion.Z * z_temp;
    temp_bno.quaternion.Z = bno->quaternion.W * w_temp + bno->quaternion.X * x_temp - bno->quaternion.Y * y_temp + bno->quaternion.Z * z_temp;

    normalize_quat(&temp_bno);
}


void ekf_update(tAHRS* ahrs, tBNO055* bno) {
    lla_to_ecef(&ahrs->position);
    update_position_ned(ahrs);

    Y[0] = ahrs->position.ned.north - X[0];
    Y[1] = ahrs->position.ned.east  - X[1];
    Y[2] = ahrs->position.ned.down  - X[2];
    Y[3] = ahrs->velocity.ned.north - X[3];
    Y[4] = ahrs->velocity.ned.east  - X[4];
    Y[5] = ahrs->velocity.ned.down  - X[5];
    Y[6] = bno->eulerf.R - X[6];
    Y[7] = bno->eulerf.P - X[7];
    Y[8] = bno->eulerf.H - X[8];

    // S = HPH^T + R
    float S[9][9] = {0};
    float HP[9][15] = {0};
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 15; j++) {
            for (int k = 0; k < 15; k++) {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            for (int k = 0; k < 15; k++) {
                S[i][j] += HP[i][k] * H[j][k];
            }
            S[i][j] += R[i][j];
        }
    }

    // K = PH^T * inv(S) -> assume S is diagonal for now
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 9; j++) {
            float sum = 0.0f;
            for (int k = 0; k < 15; k++) {
                sum += P[i][k] * H[j][k];
            }
            K[i][j] = sum / (S[j][j] + 1e-6f);
        }
    }

    // X = X + K * Y
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 9; j++) {
            X[i] += K[i][j] * Y[j];
        }
    }

    // P = (I - KH) P (I - KH)^T + K R K^T
    float KH[15][15] = {0};
    float temp[15][15] = {0};
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            for (int k = 0; k < 9; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
            temp[i][j] = I[i][j] - KH[i][j];
        }
    }

    float newP[15][15] = {0};
    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            for (int k = 0; k < 15; k++) {
                newP[i][j] += temp[i][k] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 15; i++) {
        for (int j = 0; j < 15; j++) {
            P[i][j] = newP[i][j];
        }
    }

    //ahrs->telemetry.roll = X[6];
    //ahrs->telemetry.pitch = X[7];
    //ahrs->telemetry.head = X[8];

    // Basit lineer blend (daha iyisi: quaternion fark + çarpma)
    /*
    ahrs->telemetry.q0 = ((1.0f - alpha) * temp_bno.quaternion.W) + (alpha * bno->quaternion.W);
    ahrs->telemetry.q1 = ((1.0f - alpha) * temp_bno.quaternion.X) + (alpha * bno->quaternion.X);
    ahrs->telemetry.q2 = ((1.0f - alpha) * temp_bno.quaternion.Y) + (alpha * bno->quaternion.Y);
    ahrs->telemetry.q3 = ((1.0f - alpha) * temp_bno.quaternion.Z) + (alpha * bno->quaternion.Z);
	*/

    //normalize_quat(&temp_bno);
    //quaternion_to_euler(ahrs, &temp_bno);


	ahrs->telemetry.roll = kalman_filter_roll(&kalman_roll, &(bno->accelf), bno->gyrof.X, 0.01f);
	ahrs->telemetry.pitch = kalman_filter_pitch(&kalman_pitch, &(bno->accelf), bno->gyrof.Y, 0.01f);
	ahrs->telemetry.head = bno->eulerf.H;

	euler2quaternion(ahrs);

}

void ekf_task(t_gps* gps, tBNO055* bno, tAHRS* ahrs){

	if((gps->receive_check == true) && (init_check == false)){
		if((gps->gga.location.latitude > 10.00) && (gps->gga.location.longitude > 10.00)){

			ekf_init(ahrs, bno);
			init_check = true;
		}
	}


	ekf_predict(ahrs, bno, 0.1f);	// dt = 0.01f
	ekf_update(ahrs, bno);

}


