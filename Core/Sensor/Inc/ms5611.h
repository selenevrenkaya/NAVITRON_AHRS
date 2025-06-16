/*
 * ms5611.h
 *
 *  Created on: May 28, 2025
 *      Author: selenevrenkaya
 */

#include <stdint.h>
#include"stdbool.h"
#include "math.h"

#include "i2c_interface.h"
#include "stm32f4xx.h"

#include "ahrs.h"

#ifndef SENSOR_INC_MS5611_H_
#define SENSOR_INC_MS5611_H_


/* for calculating height */
#define T0 	288.15    // Temel sıcaklık (K)
#define L 	0.0065     // Sıcaklık gradyanı (K/m)
#define P0 	1006 // Deniz seviyesindeki basınç (mbar)
#define c 	287.05     // Gaz sabiti (J/(kg·K))
#define g 	9.80665    // Yerçekimi ivmesi (m/s²)
#define M 	0.0289644  // Havadan kütle (kg/mol)


/* register addresses */
#define MS5611_I2C_ADDR			0x77	/* when pin csb is low 	*/
#define MS5611_I2C_ADDR2		0x76	/* when pin csb is high */


/* register definitions */
#define MS5611_CMD_ADC_READ		0x00
#define MS5611_CMD_CONVERT_D1	0x40	// convert pressure
#define MS5611_CMD_CONVERT_D2	0x50	// convert temperature
#define MS5611_CMD_RESET		0x1E
#define MS5611_CMD_READ_PROM	0xA2	// 6 calibration values are stored from 0xA2 to 0xAC with interval 2


/* enumeration */
enum MS5611_OSR {
	MS5611_OSR_256 = 0,
	MS5611_OSR_512,
	MS5611_OSR_1024,
	MS5611_OSR_2048,
	MS5611_OSR_4096,
};


typedef struct ms5611{
	I2C_HandleTypeDef* hi2c;

	float temp;
	float pressure;

}t_ms5611;




/* functions */
void ms5611_init();

uint8_t ms5611_read_i2c(uint8_t register_address,uint8_t length,uint8_t* output);
uint8_t ms5611_write_i2c(uint8_t register_address,uint8_t length,uint8_t* input);

void ms5611_osr_select(enum MS5611_OSR osr);



bool ms5611_update_pressure();
bool ms5611_update_temperature();

void ms5611_task(tAHRS* ahrs);

float pressure_2_high();

double ms5611_get_temperature();
double ms5611_get_pressure();



#endif /* SENSOR_INC_MS5611_H_ */
