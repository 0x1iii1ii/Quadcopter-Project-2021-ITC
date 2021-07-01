/*
 * MPU6050_ITC.c
 *
 *  Created on: Dec 27, 2020
 *      Author: LiiSeng
 *     Base on: MPU6050_tockn for arduino
 *     Link to github: https://github.com/tockn/MPU6050_tockn
 */

#include <MPU6050_HMC5883L_ITC.h>

float accCoef = 0.05f;
float gyroCoef = 0.95f;
float MagCoef = 0.05f;

const float tempScale = 333.87f;
const float tempOffset = 21.0f;
const float G = 9.807f;
const float PI_Rad = 3.1415f;
const float PI_Deg = 180.0f;

int numSamples = 2000;
uint8_t check[1];
uint8_t buffer[6];

//MPU6050 R/W
/* writes a byte to MPU6050 register given a register address and data */
void WriteRegister(uint8_t memAddress, uint8_t data)
{
	HAL_I2C_Mem_Write(&MPU6050_I2C, MPU6050_ADDR, memAddress, 1, &data, 1, 1000);
	HAL_Delay(10);
}

/* reads registers from MPU6050 given a starting register address, number of bytes, and a pointer to store data */
void ReadRegister(uint8_t memAddress, uint16_t bytes, uint8_t* data)
{
	HAL_I2C_Mem_Read(&MPU6050_I2C, MPU6050_ADDR, memAddress, 1, data, bytes, 1000);
}

//HMC5883L R/W
void I2Cdev_writeByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	HAL_I2C_Mem_Write(&HMC5883L_I2C, dev_addr, reg_addr, 1, &data, 1, 1000);
	HAL_Delay(10);
}

void I2Cdev_readBytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t* data)
{
	HAL_I2C_Mem_Read(&HMC5883L_I2C, dev_addr, reg_addr, 1, data, len, 1000);
}

void I2Cdev_readByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data)
{
	HAL_I2C_Mem_Read(&HMC5883L_I2C, dev_addr, reg_addr, 1, data, 1, 1000);
}

/* Initialize MPU6050 */
void MPU6050_Init()
{
	// reset the MPU6050
	WriteRegister(PWR_MGMT_1, PWR_RESET);

	// wait for MPU6050 to come back up
	HAL_Delay(100);

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

	MPU6050_CaliGyro();
	}

	preInterval = HAL_GetTick();
}

/* Initialize HMC5883L */
void HMC5883L_Init()
{
    // write CONFIG_A register
    I2Cdev_writeByte(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_A,HMC5883l_Enable_A);

    // write CONFIG_B register
    I2Cdev_writeByte(HMC5883L_ADDRESS, HMC5883L_RA_CONFIG_B,HMC5883l_Enable_B);

    // write MODE register
    I2Cdev_writeByte(HMC5883L_ADDRESS, HMC5883L_RA_MODE,HMC5883L_MODE_CONTINUOUS);

}

uint8_t HMC5883L_ID() {

	uint8_t HMC5883L_ID;
	I2Cdev_readBytes(HMC5883L_ADDRESS, HMC5883L_RA_ID_A, 3, buffer);

    if ( buffer[0] == 'H' && buffer[1] == '4' && buffer[2] == '3')
    {
    	HMC5883L_ID = 0x44;
    }
    return HMC5883L_ID;
}

/////////////// HMC5883L Data Read /////////////////

int HMC5883L_getCompass()
{
	int16_t Mx, My,Mz;

    I2Cdev_readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer);

    Mx = (((int16_t)buffer[0]) << 8) | buffer[1];
    My = (((int16_t)buffer[4]) << 8) | buffer[5];
    Mz = (((int16_t)buffer[2]) << 8) | buffer[3];

    float roll = angleX;
    float pitch = angleY;
    // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
    float cosRoll = cos(roll);
    float sinRoll = sin(roll);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);
    //tilt compensation pitch and roll
    float fixedXh = (Mx * cosPitch) + (Mz * sinPitch);
    float fixedYh = (Mx * sinRoll * sinPitch) + (My * cosRoll) - (Mz * sinRoll * cosPitch);

    // Calculate heading  then correct for signs of axis.
    float heading = atan2(fixedYh, fixedXh);

    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -0° 41’ which is -0.6833333 Degrees, or (which we need) -0.01192641073 radians, I will use -0.0457
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = -0.01192641073;
    heading += declinationAngle;

    // Correct for when signs are reversed.
    if(heading < 0)
    heading += 2*PI_Rad;

    // Check for wrap due to addition of declination.
    if(heading > 2*PI_Rad)
    heading -= 2*PI_Rad;

    // Convert radians to degrees for readability.
    float headingDegrees = heading * PI_Deg/PI_Rad;

    return headingDegrees;
}

void HMC5883L_getHeading(int16_t *x, int16_t *y, int16_t *z)
{
    I2Cdev_readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[4]) << 8) | buffer[5];
    *z = (((int16_t)buffer[2]) << 8) | buffer[3];
}

int16_t HMC5883L_getHeadingX()
{
    I2Cdev_readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t HMC5883L_getHeadingY()
{
    I2Cdev_readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer);
    return (((int16_t)buffer[4]) << 8) | buffer[5];
}

int16_t HMC5883L_getHeadingZ()
{
    I2Cdev_readBytes(HMC5883L_ADDRESS, HMC5883L_RA_DATAX_H, 6, buffer);
    return (((int16_t)buffer[2]) << 8) | buffer[3];
}

////////////////////////////////////////////////////////////////////

//////////////////////// MPU6050 Data Read /////////////////////////

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
	angleZ = (gyroCoef * (angleZ + gyroZ * interval)) + (MagCoef * (interval * (HMC5883L_getCompass() - 250)));
	//angleZ = angleGyroZ;
	if (angleZ < 0)
	{
		angleZ +=360;
	}
	if (angleZ > 360)
	{
		angleZ -=360;
	}
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

uint8_t MPU6050_ID(){ return check[0]; };

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

////////////////////////////////////////////////////////////////////
