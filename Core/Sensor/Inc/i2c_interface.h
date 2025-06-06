/*
 * i2c_interface.h
 *
 *  Created on: Mar 29, 2025
 *      Author: selenevrenkaya
 */

#ifndef SENSOR_INC_I2C_INTERFACE_H_
#define SENSOR_INC_I2C_INTERFACE_H_

#include "main.h"


typedef enum
{
	I2C_SUCCESS = 	(0x00U),
	I2C_FAIL = 		(0x01U),
	I2C_BUSY =	 	(0x02U),
	I2C_TIMEOUT =	(0x03U)
}tI2C_Status;



#endif /* SENSOR_INC_I2C_INTERFACE_H_ */
