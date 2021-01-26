/*
 * MPU6050_ITC.c
 *
 *  Created on: Dec 27, 2020
 *      Author: LiiSeng
 *     Base on: MPU6050_tockn for arduino
 */

#include <MPU6050_ITC.h>

float accCoef = 0.04f;
float gyroCoef = 0.96f;

const float tempScale = 333.87f;
const float tempOffset = 21.0f;
const float G = 9.807f;
const float PI_Rad = 3.1415f;
const float PI_Deg = 180.0f;

int numSamples = 3000;

/* writes a byte to MPU6050 register given a register address and data */
void WriteRegister(uint8_t memAddress, uint8_t data)
{
	HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR, memAddress, 1, &data, 1, 1000);
	HAL_Delay(10);
}

/* reads registers from MPU6050 given a starting register address, number of bytes, and a pointer to store data */
void ReadRegister(uint8_t memAddress, uint16_t bytes, uint8_t* data)
{
	HAL_I2C_Mem_Read (&MPU6050_I2C, MPU6050_ADDR, memAddress, 1, data, bytes, 1000);
}

/* Initialize MPU6050 */
void MPU6050_Init()
{
	// reset the MPU6050
	WriteRegister(PWR_MGMT_1, PWR_RESET);

	// wait for MPU6050 to come back up
	HAL_Delay(100);

	uint8_t check[1];
	// check device ID WHO_AM_I of MPU6050
	ReadRegister(WHO_AM_I, 1, check);

	if (check[0] == 0x68)  // 0x68 will be returned by the sensor if it's ready and response.
	{

	//Set DATA RATE OUT = Internal Rate Out by writing 0x00 to SMPLRT_DIV register
	//SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)
	WriteRegister(SMPLRT_DIV, SMPLRT_DIV_RATE);

	//disable DLPF filter
	WriteRegister(CONFIG, DLPF);

	// Set accelerometer configuration in ACCEL_CONFIG Register
	WriteRegister(ACCEL_CONFIG, ACCEL_FS_SEL_2G);

	// Set Gyroscopic configuration in GYRO_CONFIG Register
	WriteRegister(GYRO_CONFIG, GYRO_FS_SEL_500DPS);

	//select clock source 8Mhz to gyro
	WriteRegister(PWR_MGMT_1, CLKSEL);

	}

	MPU6050_CaliGyro();
	preInterval = HAL_GetTick();
}

void MPU6050_CaliGyro()
{

	  float x = 0, y = 0, z = 0;
	  int16_t rx, ry, rz;
	  //read first numSample to find the average
	  for(int i = 0; i < numSamples; i++)
	  {
		uint8_t bufferData[6];
		// Read 6 BYTES of data starting from GYRO_XOUT_H register
		ReadRegister(GYRO_OUT, 6, bufferData);
		//combine 8it output data into 16it
		rx= (int16_t)(bufferData[0] << 8 | bufferData [1]);
		ry = (int16_t)(bufferData[2] << 8 | bufferData [3]);
		rz = (int16_t)(bufferData[4] << 8 | bufferData [5]);
		//convert raw output into DPS at scale 500 degree/s divide by 65.5 LBS/degree/s
	    x += ((float)rx) / 65.5;
	    y += ((float)ry) / 65.5;
	    z += ((float)rz) / 65.5;
	  }
	  //devide by numSample for average
	  gyroXoffset = x / numSamples;
	  gyroYoffset = y / numSamples;
	  gyroZoffset = z / numSamples;

}

void MPU6050_GetAll()
{
	uint8_t bufferData[20];

	// grab the data from the MPU6050
	ReadRegister(ACCEL_OUT, 20, bufferData);

	// combine into 16 bit raw values
	rawAccX = (((int16_t)bufferData[0]) << 8) | bufferData[1];
	rawAccY = (((int16_t)bufferData[2]) << 8) | bufferData[3];
	rawAccZ = (((int16_t)bufferData[4]) << 8) | bufferData[5];
	rawTemp = (((int16_t)bufferData[6]) << 8) | bufferData[7];
	rawGyroX = (((int16_t)bufferData[8]) << 8) | bufferData[9];
	rawGyroY = (((int16_t)bufferData[10]) << 8) | bufferData[11];
	rawGyroZ = (((int16_t)bufferData[12]) << 8) | bufferData[13];

	//temperature in degree C
	temp = ((((float) rawTemp) - tempOffset)/tempScale) + tempOffset;

	//divide raw to get 1G
	accX = ((float)rawAccX) / 16384.0;
	accY = ((float)rawAccY) / 16384.0;
	accZ = ((float)rawAccZ) / 16384.0;

	//divide raw to get Dps
	gyroX = ((float)rawGyroX) / 65.5;
	gyroY = ((float)rawGyroY) / 65.5;
	gyroZ = ((float)rawGyroZ) / 65.5;

	//1G to angle degree X and Y
	angleAccX = atan2(accY, accZ + abs(accX)) * PI_Deg / PI_Rad;
	angleAccY = atan2(accX, accZ + abs(accY)) * -1 * PI_Deg / PI_Rad;

	//offset to 0 with gyro offset
	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	//time per-loop
	interval = (HAL_GetTick() - preInterval) * 0.001;

	//Dps to degree
	angleGyroX += gyroX * interval;
	angleGyroY += gyroY * interval;
	angleGyroZ += gyroZ * interval;

	//Complementary filter
	angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
	angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
	angleZ = angleGyroZ;

	preInterval = HAL_GetTick();

}

float getGyroXoffset(){ return gyroXoffset; };
float getGyroYoffset(){ return gyroYoffset; };
float getGyroZoffset(){ return gyroZoffset; };

int16_t getRawAccX(){ return rawAccX; };
int16_t getRawAccY(){ return rawAccY; };
int16_t getRawAccZ(){ return rawAccZ; };

int16_t getRawGyroX(){ return rawGyroX; };
int16_t getRawGyroY(){ return rawGyroY; };
int16_t getRawGyroZ(){ return rawGyroZ; };

int16_t getRawTemp(){ return rawTemp; };

float getAccX(){ return accX; };
float getAccY(){ return accY; };
float getAccZ(){ return accZ; };

float getGyroX(){ return gyroX; };
float getGyroY(){ return gyroY; };
float getGyroZ(){ return gyroZ; };

float getTemp(){ return temp; };

float getAccAngleX(){ return angleAccX; };
float getAccAngleY(){ return angleAccY; };

float getGyroAngleX(){ return angleGyroX; };
float getGyroAngleY(){ return angleGyroY; };
float getGyroAngleZ(){ return angleGyroZ; };

float getAngleX(){ return angleX; };
float getAngleY(){ return angleY; };
float getAngleZ(){ return angleZ; };
