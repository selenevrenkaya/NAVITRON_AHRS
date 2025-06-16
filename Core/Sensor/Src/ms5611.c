/*
 * ms5611.c
 *
 *  Created on: May 28, 2025
 *      Author: selenevrenkaya
 */

#include "ms5611.h"
#include "ahrs.h"


#define NUM_CALIBRATION_DATA 6



extern t_ms5611 ms5611;


uint16_t calib_data[NUM_CALIBRATION_DATA]; // 6 factory calibration data
uint32_t raw_pressure, raw_temperature;

enum MS5611_OSR selected_osr = MS5611_OSR_4096;



/**
 * Initialize MS5611: read and store factory calibration data.
 */
void ms5611_init(){

	/* read 6 factory calibration data */
	for(int i = 0; i < NUM_CALIBRATION_DATA; i++){
		uint8_t buffer[2];

		uint8_t reg_addr = MS5611_CMD_READ_PROM + (i << 1);		// interval 2
		ms5611_read_i2c(reg_addr, 2, buffer);

		calib_data[i] = (uint16_t)(buffer[0] << 8 | buffer[1]);

	}
}


/**
 * read from MS5611 with default I2C address
 * @param register_address register/command to request data
 * @param length length of bytes to request from MS5611
 * @param output output data
 * @return I2C_FAIL, 0(I2C_SUCCESS) = success
 */
uint8_t ms5611_read_i2c(uint8_t register_address,uint8_t length,uint8_t* output){

	if(HAL_OK != HAL_I2C_Mem_Read(ms5611.hi2c, (MS5611_I2C_ADDR << 1), register_address, sizeof(register_address), output, length, 10)){
		return I2C_FAIL;
	}
	else	return I2C_SUCCESS;

}

/**
 * Write to MS5611 with default I2C address
 * @param register_address register/command to send
 * @param length length of bytes to write to MS5611
 * @param output buffer to hold data to be sent
 * @return HAL_STATUS, 0(HAL_OK) = success
 */
uint8_t ms5611_write_i2c(uint8_t register_address,uint8_t length,uint8_t* input){

	if(HAL_OK != HAL_I2C_Mem_Write(ms5611.hi2c, (MS5611_I2C_ADDR << 1), register_address, sizeof(register_address), input, length, 10)){
		return I2C_FAIL;
	}
	else	return I2C_SUCCESS;

}



/**
 * set ADC resolution, from MS5611_OSR_256 to MS5611_OSR_4096
 */
void ms5611_osr_select(enum MS5611_OSR osr){
	selected_osr = osr;
}



/**
 * Read raw pressure from MS5611.
 */
bool ms5611_update_pressure(){
	uint8_t buffer[3] = {0x00,0x00,0x00};

	ms5611_osr_select(MS5611_OSR_4096);
	if(I2C_SUCCESS != ms5611_write_i2c(MS5611_CMD_CONVERT_D1 | (selected_osr << 1), 0, buffer)){
		return false;
	}

	HAL_Delay(10);//time delay necessary for ADC to convert, must be >= 9.02ms

	if(I2C_SUCCESS != ms5611_read_i2c(MS5611_CMD_ADC_READ, 3, buffer)){
		return false;
	}


	raw_pressure = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
	return true;

}



/**
 * Read raw temperature from MS5611.
 */
bool ms5611_update_temperature(){
	uint8_t buffer[3] = {0x00,0x00,0x00};

	ms5611_osr_select(MS5611_OSR_4096);
	if(I2C_SUCCESS != ms5611_write_i2c(MS5611_CMD_CONVERT_D2 | (selected_osr << 1), 0, buffer)){
		return false;
	}

	HAL_Delay(10);//time delay necessary for ADC to convert, must be >= 9.02ms

	if(I2C_SUCCESS != ms5611_read_i2c(MS5611_CMD_ADC_READ, 3, buffer)){
		return false;
	}

	raw_temperature = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2]);
	return true;
}



/**
 *	Read raw temperature and pressure from MS5611
 */
void ms5611_task(tAHRS* ahrs){

	if(ms5611_update_temperature() == true){
		if(ms5611_update_pressure() == true){

			ms5611.temp = ms5611_get_temperature();
			ms5611.pressure = ms5611_get_pressure();

		}
	}

	ahrs->telemetry.temperature = (float)(ms5611.temp);
	ahrs->telemetry.pressure = (int16_t)(ms5611.pressure);

	ahrs->telemetry.height = pressure_2_high();

}


/* convert pressure to height info */
float pressure_2_high(){
    //float height = (T0 / L) * (pow(ms5611.pressure / P0, (1 / 5.255)) - 1);
	float height = 44330.0f * (1.0f - powf(ms5611.pressure / P0, 0.190284f));

	return height;

}


/**
 * Get calibrated temperature, unit: Celsius degrees
 * @return calibrated temperature
 */
double ms5611_get_temperature(){
	uint32_t dT = raw_temperature - ((uint32_t)calib_data[4] * 256);
	double TEMP = 2000.0 + dT * (calib_data[5] / (8388608.0));//unit 0.01 C

	double T2=0;
	if (TEMP < 2000){
		//temperature < 20 Celsius
		T2 = dT * (dT / (2147483648.0));
	}

	TEMP = TEMP - T2;
	return TEMP / 100;
}



/**
 * Get calibrated pressure, unit: mBar
 * @return calibrated pressure
 */
double ms5611_get_pressure(){

	uint32_t dT = raw_temperature - ((uint32_t)calib_data[4] * 256);
	double TEMP = 2000.0 + dT * (calib_data[5] / (8388608.0));//unit 0.01 C

	double OFF = calib_data[1] * (65536) + calib_data[3] * dT / (128);
	double SENS = calib_data[0] * (32768) + calib_data[2] * dT / (256);

	double P = (raw_pressure * SENS / (2097152.0) - OFF) / (32768);//unit 0.01mbar

	double T2=0, OFF2=0, SENS2=0;
	if (TEMP < 2000){
		//temperature < 20 Celsius
		T2 = dT * dT / (2147483648);
		OFF2 = 5 * (TEMP-2000) * (TEMP-2000) / 2;
		SENS2 = 5 * (TEMP-2000) * (TEMP-2000) / 4;

		if (TEMP < -1500){
			//temperature < -15 Celsius
			OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
			SENS2 = SENS2 + 11/2 * (TEMP + 1500) * (TEMP + 1500);
		}
	}

	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	P = (raw_pressure * SENS / (2097152.0) - OFF) / (32768);//unit mbar
	return P / 100;//unit mbar
}

