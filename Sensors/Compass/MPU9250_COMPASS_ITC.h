/*
 * MPU9250_COMPASS_ITC.h
 *
 *  Created on: Jan 26, 2021
 *      Author: LeeSeng
 */
#include "stm32f4xx_hal.h"
#include "math.h"

#ifndef MPU9250_COMPASS_ITC_H_
#define MPU9250_COMPASS_ITC_H_

//i2c port
I2C_HandleTypeDef hi2c1;
#define AK8963_I2C					hi2c1 //Can be change to your desire I2C port in this case I2C1 was used.
#define MPU9250_I2C					hi2c1

#define MPU9250_ADDR 				0x68 << 1
#define MPU9250_INT_PIN_CFG  		0x37		//bypass enable

// AK8963 registers

#define AK8963_ADDR  				0x0C << 1 	//AK8963 address
#define AK8963_WHO_AM_I  			0x00		//return 0x48
#define AK8963_STAT_REG				0x02		//return 0x01 if ready else 0x00
#define AK8963_HXL  				0x03		//start data addr

#define AK8963_CNTL1  				0x0A
#define AK8963_CNTL1_PWR_DOWN  		0x00
#define AK8963_CNTL1_SINGLE_MEAS 	0x11
#define AK8963_CNTL1_MEAS1  		0x12
#define AK8963_CNTL1_MEAS2  		0x16
#define AK8963_CNTL1_FUSE_ROM  		0x0F

#define AK8963_CNTL2  				0x0B
#define AK8963_SRST_RESET  			0x01
#define AK8963_ASA  				0x10

float rawMagX, rawMagY, rawMagZ, magX, magY, magZ;
float angleMagZFiltered, angleMagZ;

void AK8963_Init();
void AK8963_GetMag();


#endif /* MPU9250_COMPASS_ITC_H_ */
