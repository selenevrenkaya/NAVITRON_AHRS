/*
 * ahrs.h
 *
 *  Created on: Jun 1, 2025
 *      Author: selenevrenkaya
 */

#ifndef INC_AHRS_H_
#define INC_AHRS_H_

#include "stdint.h"


typedef struct NED_Velocity{
	float north;
	float east;
	float down;

}tNED_Velocity;


typedef struct Velocity{
	tNED_Velocity ned;

}tVelocity;


typedef struct NED_Position{
	float north;
	float east;
	float down;

}tNED_Position;


typedef struct ECEF_Position{
	float x;
	float y;
	float z;

}tECEF_Position;


typedef struct Position{
	double latitude;		/* gps */
	double longitude;				// degree -->  can be chanced to radian (?)
	float altitude;

	tNED_Position ned;

	tECEF_Position ecef;

}tPosition;


typedef struct Telemetry{
	float roll;	/* euler angles */
	float pitch;
	float head;

	float q0;		/* quaternions */
	float q1;
	float q2;
	float q3;

	float aX;	/* accel in x, y, z coordinates */
	float aY;
	float aZ;

	float aN_ned;	/* accel in earth coordinates */
	float aE_ned;
	float aD_ned;

	float temperature;	/* in degree celcius */
	int16_t pressure;		/* unit mbar */
	float height;			/* altitude info from barometer (meter) */

}tTelemetry;



typedef struct AHRS{
	tTelemetry telemetry;
	tPosition position;
	tVelocity velocity;

}tAHRS;




#endif /* INC_AHRS_H_ */


