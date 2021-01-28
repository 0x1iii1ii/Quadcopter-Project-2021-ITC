/*
 * BME280_ITC.h
 *
 *  Created on: Jan 28, 2021
 *      Author: LiiSeng
 *      This library is modified from Seeed-Studio. I just ported it to STM32
 *      link to github: https://github.com/Seeed-Studio/Grove_BME280
 */

#ifndef BME280_ITC_H_
#define BME280_ITC_H_

#include "math.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"

//i2c port
I2C_HandleTypeDef hi2c1;

#define BME280_I2C			hi2c1 //Can be change to your desire I2C port in this case I2C1 was used.

#define BME280_ADDRESS   	 0x76 << 1 //SDO to GND, 0x77 SDO to Vcc

#define BME280_REG_DIG_T1    0x88
#define BME280_REG_DIG_T2    0x8A
#define BME280_REG_DIG_T3    0x8C

#define BME280_REG_DIG_P1    0x8E
#define BME280_REG_DIG_P2    0x90
#define BME280_REG_DIG_P3    0x92
#define BME280_REG_DIG_P4    0x94
#define BME280_REG_DIG_P5    0x96
#define BME280_REG_DIG_P6    0x98
#define BME280_REG_DIG_P7    0x9A
#define BME280_REG_DIG_P8    0x9C
#define BME280_REG_DIG_P9    0x9E

#define BME280_REG_DIG_H1    0xA1
#define BME280_REG_DIG_H2    0xE1
#define BME280_REG_DIG_H3    0xE3
#define BME280_REG_DIG_H4    0xE4
#define BME280_REG_DIG_H5    0xE5
#define BME280_REG_DIG_H6    0xE7

#define BME280_REG_CHIPID          0xD0
#define BME280_REG_VERSION         0xD1
#define BME280_REG_SOFTRESET       0xE0

#define BME280_REG_CAL26           0xE1

#define BME280_REG_CONTROLHUMID    0xF2
#define BME280_REG_CONTROL         0xF4
#define BME280_REG_CONFIG          0xF5
#define BME280_REG_PRESSUREDATA    0xF7
#define BME280_REG_TEMPDATA        0xFA
#define BME280_REG_HUMIDITYDATA    0xFD


	//public functions
    void BME280_Init();
    float BME280_GetTemperature(void);
    uint32_t BME280_GetPressure(void);
    uint32_t BME280_GetHumidity(void);
    float BME280_CalcAltitude(float pressure);

    // Calibration data
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t  dig_H6;
    int32_t t_fine;

    // private functions
    uint8_t BME280Read8(uint8_t reg);
    uint16_t BME280Read16(uint8_t reg);
    uint16_t BME280Read16LE(uint8_t reg);
    int16_t BME280ReadS16(uint8_t reg);
    int16_t BME280ReadS16LE(uint8_t reg);
    uint32_t BME280Read24(uint8_t reg);
    void BME280_writeRegister(uint8_t reg, uint8_t val);


#endif /* BME280_ITC_H_ */
