/*
 * MPU9250_COMPASS_ITC.c
 *
 *  Created on: Jan 26, 2021
 *      Author: LeeSeng
 */

#include "MPU9250_COMPASS_ITC.h"

static uint8_t mag_adjust[3];

const float G = 9.807f;
const float PI_Rad = 3.14159f;
const float PI_Deg = 180.0f;
float declination = 0.57083f;//degree
int16_t  headingAdjust = 0;

/* writes a byte to MPU9250 register given a register address and data */
void WriteMPU9250Register(uint8_t memAddress, uint8_t data)
{
	HAL_I2C_Mem_Write(&MPU9250_I2C, MPU9250_ADDR, memAddress, 1, &data, 1, 1000);
	HAL_Delay(10);
}

/* writes a byte to AK8963 register given a register address and data */
void WriteRegister(uint8_t memAddress, uint8_t data)
{
	HAL_I2C_Mem_Write(&AK8963_I2C, AK8963_ADDR, memAddress, 1, &data, 1, 1000);
	HAL_Delay(10);
}

/* reads registers from AK8963 given a starting register address, number of bytes, and a pointer to store data */
void ReadRegister(uint8_t memAddress, uint16_t bytes, uint8_t* data)
{
	HAL_I2C_Mem_Read(&AK8963_I2C, AK8963_ADDR, memAddress, 1, data, bytes, 1000);
}

void AK8963_Init()
{

	// Write enable i2c direclly to AK8963
	WriteMPU9250Register(MPU9250_INT_PIN_CFG, 0x02);

	// set AK8963 to Power Down
	WriteRegister(AK8963_CNTL1, AK8963_CNTL1_PWR_DOWN);

	// give some time to come back up
	HAL_Delay(10);

	// reset the AK8963
	WriteRegister(AK8963_CNTL2, AK8963_SRST_RESET);

	// give some time to come back up
	HAL_Delay(10);

	//check if communication is okay
	uint8_t check[1];

	//Read who am i reg expected 0x48 in return
	ReadRegister(AK8963_WHO_AM_I, 1, check);

	if (check[0] == 0x48 ) // return 0x48 if everything good
	{

		// set AK8963 to FUSE ROM access
		WriteRegister(AK8963_CNTL1,AK8963_CNTL1_FUSE_ROM);

		// long wait between AK8963 mode changes
		HAL_Delay(10);

		// read the AK8963 ASA registers and compute magnetometer scale factors
		ReadRegister(AK8963_ASA, 3, mag_adjust);

		// set AK8963 to Power Down
		WriteRegister(AK8963_CNTL1,AK8963_CNTL1_PWR_DOWN);

		// long wait between AK8963 mode changes
		HAL_Delay(10);

		// set AK8963 to 16 bit resolution, and set mode to single measurement
		WriteRegister(AK8963_CNTL1,AK8963_CNTL1_SINGLE_MEAS);

		// long wait between AK8963 mode changes
		HAL_Delay(10);

		//check data if it ready to read
		//ReadRegister(AK8963_STAT_REG, 1, check);

		//wai till it ready
		//while (check[0] != 0x01)
		//{
		//	;
		//}

	}
}

void AK8963_GetMag()
{
	uint8_t bufferData[6];
	// grab the data from the MPU9250
	ReadRegister(AK8963_HXL, 6, bufferData);

	//read raw
	rawMagX = (((int16_t)bufferData[1]) << 8) | bufferData[0];
	rawMagY = (((int16_t)bufferData[3]) << 8) | bufferData[2];
	rawMagZ = (((int16_t)bufferData[5]) << 8) | bufferData[4];

	//convert to uT
	magX = (float)((int16_t)rawMagX * ((float)(mag_adjust[0] - 128) / 256.0f + 1.0f));
	magY = (float)((int16_t)rawMagY * ((float)(mag_adjust[1] - 128) / 256.0f + 1.0f));
	magZ = (float)((int16_t)rawMagZ * ((float)(mag_adjust[2] - 128) / 256.0f + 1.0f));

	//convert to angle
	//angleMagZ = headingAdjust + atan2(magY,magX) * PI_Deg / PI_Rad;

	//add fix heading base on location
	//angleMagZ += declination;

	// Smoothing the output angle / Low pass filter
	//angleMagZFiltered = angleMagZFiltered * 0.85 + angleMagZ * 0.15;

}

