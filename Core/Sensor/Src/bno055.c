/*
 * bno055.c
 *
 *  Created on: Mar 29, 2025
 *      Author: selenevrenkaya
 */

#include "bno055.h"
#include "ahrs.h"

#define BNO_MAX_ERR		(10)
#define IMU_CALIBRETED  (1)

extern tAHRS ahrs;

const static tCalibrationVals calib_data = {
    .accel_offset_x = 0,
    .accel_offset_y = 0,
    .accel_offset_z = 0,
    .mag_offset_x = 0,
    .mag_offset_y = 0,
    .mag_offset_z = 0,
    .gyro_offset_x = 0,
    .gyro_offset_y = 0,
    .gyro_offset_z = 0,
    .accel_radius = 0,
    .mag_radius = 480
};


static uint32_t current_ms, prev_ms = 0;
static inline void BNO055_SetDelay(tBNO055* bno, uint32_t ms);
static tI2C_Status BNO055_ProcessAccel(tBNO055* bno);
static void BNO055_Euler2Quaternion(tBNO055* bno);
static tI2C_Status BNO055_ProcessEuler(tBNO055* bno);


tI2C_Status BNO055_Init(tBNO055* bno, I2C_HandleTypeDef* hi2c){
	if(bno == NULL || hi2c ==  NULL){
		return I2C_FAIL;
	}

	memset(bno, 0, sizeof((tBNO055)*bno));
	bno->taskPeriod = BNO055_TASK_PERIOD;
	bno->hi2c= hi2c;
	bno->state = 0;
	return I2C_SUCCESS;
}


tI2C_Status BNO055_SetExtClock(tBNO055* bno){
	uint8_t extClock = BNO055_USE_EXT_CLCK;
	if(HAL_OK != HAL_I2C_Mem_Write(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_SYS_TRIGGER_ADDR, 1, &extClock, 1, 2)){
		return I2C_FAIL;
	}

	return I2C_SUCCESS;
}


tI2C_Status BNO055_SetPowerMode(tBNO055* bno, eBNO055_PowerMode pmode){
	uint8_t powerMode = (uint8_t)pmode;
	if(HAL_OK != HAL_I2C_Mem_Write(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_PWR_MODE_ADDR, 1, &powerMode, 1, 2)){
		return I2C_FAIL;
	}

	return I2C_SUCCESS;
}


tI2C_Status BNO055_SetOperationMode(tBNO055* bno, eBNO055_OperationMode opmode){
	uint8_t opMode = (uint8_t)opmode;
	if(HAL_OK != HAL_I2C_Mem_Write(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_OPR_MODE_ADDR, 1, &opMode, 1, 2)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}

/* Skipped the definitions of these function for the time being. Will be addressed
 * if deemed necessary.
 *
 *
 **/
tI2C_Status BNO055_SetCalibration(tBNO055* bno){

#if (IMU_CALIBRETED)

if(HAL_OK != HAL_I2C_Mem_Write(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_ACC_OFFSET_X_LSB, 1, (uint8_t*)(tCalibrationVals*)(&calib_data), BNO055_TOTAL_CALIB_SIZE, 10)){
        return I2C_FAIL;
    }
        return I2C_SUCCESS;


#else

return I2C_SUCCESS;

#endif
}

tI2C_Status BNO055_GetCalibration(tBNO055* bno){
    if(HAL_OK != HAL_I2C_Mem_Read(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_ACC_OFFSET_X_LSB, 1, (uint8_t*)(tCalibrationVals*)(&(bno->calib)), BNO055_TOTAL_CALIB_SIZE, 10)){
        return I2C_FAIL;
    }
    return I2C_SUCCESS;

}


tI2C_Status BNO055_GetAccel(tBNO055* bno){
	if(HAL_OK != HAL_I2C_Mem_Read(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_ACCEL_DATA_X_LSB, 1, (uint8_t*)(tEuler*)(&(bno->accel)), BNO055_ACCEL_XYZ_DATA_SIZE, 2)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}


tI2C_Status BNO055_GetEuler(tBNO055* bno){
	if(HAL_OK != HAL_I2C_Mem_Read(bno->hi2c, BNO055_HAL_I2C_ADDR, BNO055_EUL_DATA_H_LSB, 1, (uint8_t*)(tEuler*)(&(bno->euler)), BNO055_EULER_HRP_DATA_SIZE, 2)){
		return I2C_FAIL;
	}
	return I2C_SUCCESS;
}


static tI2C_Status BNO055_ProcessAccel(tBNO055* bno){
	bno->accelf.X = (float)bno->accel.X / BNO055_ACCEL_DIV_MSQ;
	bno->accelf.Y = (float)bno->accel.Y / BNO055_ACCEL_DIV_MSQ;
	bno->accelf.Z = (float)bno->accel.Z / BNO055_ACCEL_DIV_MSQ;

	return I2C_SUCCESS;
}


static void BNO055_Euler2Quaternion(tBNO055* bno){
    /* degree to radian */
	float deg2rad = M_PI / 180.0;

	float yaw_rad = bno->eulerf.H * deg2rad;
	float pitch_rad = bno->eulerf.P * deg2rad;
	float roll_rad = bno->eulerf.R * deg2rad;


	/* Yari acilarin trigonometrik fonksiyonlari */
    float cy = cos(yaw_rad * 0.5);    	/* yaw (Z) */
    float sy = sin(yaw_rad * 0.5);
    float cp = cos(pitch_rad * 0.5);	/* pitch (Y) */
    float sp = sin(pitch_rad * 0.5);
    float cr = cos(roll_rad * 0.5);   	/* roll (X) */
    float sr = sin(roll_rad * 0.5);

    bno->quaternion.W = cr * cp * cy + sr * sp * sy;
    bno->quaternion.X = sr * cp * cy - cr * sp * sy;
    bno->quaternion.Y = cr * sp * cy + sr * cp * sy;
    bno->quaternion.Z = cr * cp * sy - sr * sp * cy;

	return ;
}


static tI2C_Status BNO055_ProcessEuler(tBNO055* bno){
	bno->eulerf.H = (float)bno->euler.H / BNO055_EULER_DIV_DEG;
	bno->eulerf.R = (float)bno->euler.R / BNO055_EULER_DIV_DEG;
	bno->eulerf.P = (float)bno->euler.P / BNO055_EULER_DIV_DEG;

	return I2C_SUCCESS;
}



tI2C_Status BNO055_Task(tBNO055* bno, tAHRS* ahrs){

	/* Get the current tick from HAL SysTick timer update counter */
	current_ms = uwTick;

	/* Check if the the time for task to run has come */
	if(current_ms - prev_ms < bno->taskPeriod){
		return I2C_TIMEOUT;
	}
	/* Update the previously taken tick*/
	prev_ms = current_ms;

	eBNO055_Task_States state = bno->state;
	tI2C_Status ret = I2C_FAIL;
	if(bno->errorCount > BNO_MAX_ERR){
		bno->errorCount = 0;
		state = BNO055_SET_LnRESET;
	}

	if (state == BNO055_GET_CALIB_DATA) {
	    uint8_t ret = I2C_SUCCESS;
	    ret = BNO055_GetCalibration(bno);
	    BNO055_SetDelay(bno, 650);
        if (ret == I2C_SUCCESS)
            bno->state++;
	}

	if (state == BNO055_SET_CONFIG_MODE) {
	    uint8_t ret = I2C_SUCCESS;
	    ret = BNO055_SetOperationMode(bno, BNO055_OPERATION_MODE_CONFIG);
	    BNO055_SetDelay(bno, 650);
        if (ret == I2C_SUCCESS)
            bno->state++;
	}

	else if(state == BNO055_SET_CALIB_DATA){
        ret = BNO055_SetCalibration(bno);
        BNO055_SetDelay(bno, 650);
        if (ret == I2C_SUCCESS)
            bno->state++;
    }

    else if(state == BNO055_SET_EXT_CLCK){
		ret = BNO055_SetExtClock(bno);
		BNO055_SetDelay(bno, 650);
        if (ret == I2C_SUCCESS)
            bno->state++;
	}

	else if(state == BNO055_SET_POWER_MODE){
		ret = BNO055_SetPowerMode(bno, BNO055_POWER_MODE_NORMAL);
		BNO055_SetDelay(bno, 650);
        if (ret == I2C_SUCCESS)
            bno->state++;
	}

	else if(state == BNO055_SET_OPERATION_MODE){
		ret = BNO055_SetOperationMode(bno, BNO055_OPERATION_MODE_NDOF);
		BNO055_SetDelay(bno, 650);
		if (ret == I2C_SUCCESS)
		    bno->state++;
	}

	else if(state == BNO055_GET_ACCEL){
		ret = BNO055_GetAccel(bno);
		BNO055_SetDelay(bno, BNO055_TASK_PERIOD);

        if (ret == I2C_SUCCESS){
    		ret = BNO055_ProcessAccel(bno);

    		ahrs->telemetry.aX = bno->accelf.X;
    		ahrs->telemetry.aY = bno->accelf.Y;
    		ahrs->telemetry.aZ = bno->accelf.Z;

    		if (ret == I2C_SUCCESS)
    			bno->state++;
        }

	}

	else if(state == BNO055_GET_EULER){
		ret = BNO055_GetEuler(bno);
		BNO055_SetDelay(bno, BNO055_TASK_PERIOD);

        if (ret == I2C_SUCCESS){
    		ret = BNO055_ProcessEuler(bno);
    		BNO055_Euler2Quaternion(bno);

    		ahrs->telemetry.roll = bno->eulerf.R;
    		ahrs->telemetry.pitch = bno->eulerf.P;
    		ahrs->telemetry.head = bno->eulerf.H;

    		ahrs->telemetry.q0 = bno->quaternion.W;
    		ahrs->telemetry.q1 = bno->quaternion.X;
    		ahrs->telemetry.q2 = bno->quaternion.Y;
    		ahrs->telemetry.q3 = bno->quaternion.Z;

    		if (ret == I2C_SUCCESS)
    			bno->state = BNO055_GET_ACCEL;
        }

	}


	else if(state == BNO055_SET_LnRESET){
		//HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, 0);
		BNO055_SetDelay(bno, 3);

#if (IMU_CALIBRETED == 0)
		bno->resetCount++;

#endif

		bno->state = BNO055_SET_HnRESET;
	}
	else if(state == BNO055_SET_HnRESET){
		//HAL_GPIO_WritePin(IMU_RST_GPIO_Port, IMU_RST_Pin, 1);
		BNO055_SetDelay(bno, 3);
		bno->state = BNO055_SET_CONFIG_MODE;
	}

	bno->errorCount += ret;


	return ret;
}


static inline void BNO055_SetDelay(tBNO055* bno, uint32_t ms){
	if(ms < 0 || ms > 1000)
		return ;
	bno->taskPeriod = ms;
}

/* BNO055 State update functions */
inline void BNO055_SetStateNext(tBNO055* bno){
	if(bno->state < BNO055_GET_EULER){
		bno->state++;
	}
}

inline void BNO055_SetStateDefault(tBNO055* bno){
	bno->state = 0;
}

