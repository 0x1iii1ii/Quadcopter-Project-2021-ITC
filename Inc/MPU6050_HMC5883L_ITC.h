/*
 * MPU6050_ITC.h
 *
 *  Created on: Dec 27, 2020
 *      Author: LiiSeng
 *     Base on: MPU6050_tockn for arduino
 *     Link to github: https://github.com/tockn/MPU6050_tockn
 */

#ifndef MPU6050_HMC5883L_ITC_H_
#define MPU6050_HMC5883L_ITC_H_


#include "math.h"
#include "stdlib.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"

//i2c port MPU6050
I2C_HandleTypeDef hi2c1;

#define MPU6050_I2C			hi2c1 //Can be change to your desire I2C port in this case I2C1 was used.
//i2c port HMC5883L
I2C_HandleTypeDef hi2c2;

#define HMC5883L_I2C		hi2c2 //Can be change to your desire I2C port in this case I2C3 was used.

// MPU6050 registers

#define MPU6050_ADDR 				0x68 << 1 // 7bit addr I2C 1bit LSB R/W

#define SMPLRT_DIV   				0x19
#define SMPLRT_DIV_RATE				0x00
#define CLKSEL 						0x00
#define CONFIG      				0x1A
#define DLPF      					0x00
#define WHO_AM_I     				0x75
#define PWR_MGMT_1   				0x6B
#define PWR_RESET 					0x80
#define PWR_WAKE  					0x00
#define TEMP_OUT       				0x41

#define GYRO_OUT 					0x43
#define GYRO_CONFIG  				0x1B
#define GYRO_FS_SEL_250DPS  		0x00
#define GYRO_FS_SEL_500DPS  		0x08
#define GYRO_FS_SEL_1000DPS 		0x10
#define GYRO_FS_SEL_2000DPS 		0x18

#define ACCEL_OUT 	 				0x3B
#define ACCEL_CONFIG 				0x1C
#define ACCEL_FS_SEL_2G 			0x00
#define ACCEL_FS_SEL_4G  			0x08
#define ACCEL_FS_SEL_8G  			0x10
#define ACCEL_FS_SEL_16G  			0x18

// HMC5883L registers

#define HMC5883L_ADDRESS            0x1E << 1 // 7bit addr I2C 1bit LSB R/W

#define HMC5883L_RA_CONFIG_A        0x00
#define HMC5883L_RA_CONFIG_B        0x01
#define HMC5883L_RA_MODE            0x02

#define HMC5883L_RA_DATAX_H         0x03
#define HMC5883L_RA_DATAX_L         0x04
#define HMC5883L_RA_DATAZ_H         0x05
#define HMC5883L_RA_DATAZ_L         0x06
#define HMC5883L_RA_DATAY_H         0x07
#define HMC5883L_RA_DATAY_L         0x08
#define HMC5883L_RA_STATUS          0x09

#define HMC5883l_Enable_A 			0x70
#define HMC5883l_Enable_B 			0x20
#define HMC5883l_MR 				0x00

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_RA_ID_A			0x10
#define HMC5883L_RA_ID_B			0x11
#define HMC5883L_RA_ID_C			0x12

//HMC5883L API

void HMC5883L_Init();
uint8_t HMC5883L_ID();
int HMC5883L_getCompass();
// HMC5883L DATA registers
void HMC5883L_getHeading(int16_t *x, int16_t *y, int16_t *z);
int16_t HMC5883L_getHeadingX();
int16_t HMC5883L_getHeadingY();
int16_t HMC5883L_getHeadingZ();

//MPU6050 Var
float gyroXoffset, gyroYoffset, gyroZoffset;

int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;

float angleGyroX, angleGyroY, angleGyroZ, angleAccX, angleAccY;

float angleX, angleY, angleZ;

float interval;
float preInterval;

//MPU6050 API
void MPU6050_Init();
void MPU6050_CaliGyro();
void MPU6050_GetAll();

float getGyroXoffset();
float getGyroYoffset();
float getGyroZoffset();

int16_t getRawAccX();
int16_t getRawAccY();
int16_t getRawAccZ();

int16_t getRawGyroX();
int16_t getRawGyroY();
int16_t getRawGyroZ();

int16_t getRawTemp();
uint8_t MPU6050_ID();

float getAccX();
float getAccY();
float getAccZ();

float getGyroX();
float getGyroY();
float getGyroZ();

float getTemp();

float getAccAngleX();
float getAccAngleY();

float getGyroAngleX();
float getGyroAngleY();
float getGyroAngleZ();

float getAngleX();
float getAngleY();
float getAngleZ();

#endif /* MPU6050_ITC_H_ */
