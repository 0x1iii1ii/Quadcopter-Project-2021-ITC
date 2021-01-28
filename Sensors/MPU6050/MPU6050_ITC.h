/*
 * MPU6050_ITC.h
 *
 *  Created on: Dec 27, 2020
 *      Author: LiiSeng
 *     Base on: MPU6050_tockn for arduino
 *     Link to github: https://github.com/tockn/MPU6050_tockn
 */

#ifndef MPU6050_ITC_H_
#define MPU6050_ITC_H_


#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"

//i2c port
I2C_HandleTypeDef hi2c1;

#define MPU6050_I2C			hi2c1 //Can be change to your desire I2C port in this case I2C1 was used.

// MPU6050 registers

#define MPU6050_ADDR 		0x68 << 1 // 7bit addr I2C 1bit LSB R/W

#define SMPLRT_DIV   		0x19
#define SMPLRT_DIV_RATE		0x00
#define CLKSEL 				0x00
#define CONFIG      		0x1A
#define DLPF      			0x00
#define WHO_AM_I     		0x75
#define PWR_MGMT_1   		0x6B
#define PWR_RESET 			0x80
#define PWR_WAKE  			0x00
#define TEMP_OUT       		0x41

#define GYRO_OUT 			0x43
#define GYRO_CONFIG  		0x1B
#define GYRO_FS_SEL_250DPS  0x00
#define GYRO_FS_SEL_500DPS  0x08
#define GYRO_FS_SEL_1000DPS 0x10
#define GYRO_FS_SEL_2000DPS 0x18

#define ACCEL_OUT 	 		0x3B
#define ACCEL_CONFIG 		0x1C
#define ACCEL_FS_SEL_2G 	0x00
#define ACCEL_FS_SEL_4G  	0x08
#define ACCEL_FS_SEL_8G  	0x10
#define ACCEL_FS_SEL_16G  	0x18

float gyroXoffset, gyroYoffset, gyroZoffset;

int16_t rawAccX, rawAccY, rawAccZ, rawTemp, rawGyroX, rawGyroY, rawGyroZ;

float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;

float angleGyroX, angleGyroY, angleGyroZ, angleAccX, angleAccY;

float angleX, angleY, angleZ;

float interval;
float preInterval;

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
