/*
 * checkSysOnGround.h
 *
 *  Created on: Feb 25, 2021
 *      Author: LiiSeng
 */

#ifndef CHECKSYSONGROUND_H_
#define CHECKSYSONGROUND_H_

#include "MPU6050_HMC5883L_ITC.h"
#include "BMP180.h"
#include "ReadRC6H.h"
#include "ControlButtons.h"

extern ADC_HandleTypeDef hadc1;

uint8_t errorMPU, errorBMP, errorHMC, errorBatt, errorTrans;
uint8_t errorSensors, errorPeripheral;
uint16_t batt_voltage;

float Vs, V_batt;

//resitor for divider
const uint8_t R1 = 10;
const uint8_t R2 = 3.3;
//resolution ADC 12 bit = 4096 = 3.3V input
//this case 3.0 V input thus Res12bit = 4096 x 3.0 / 3.3
//3.0 = 3723.63 thus for resolution 0.1 V ,  DAC = 3723.63 x 0.1 / 3.0
const uint16_t ResDAC = 1117.09;

//============ CHECK SENSORS ==============//
void sensorCheck() {
	if (MPU6050_ID() == 0x68) {
		errorMPU = 0;
	} else {
		errorMPU = 1;
	}
	if (BMP180_ID() == 0x55) {
		errorBMP = 0;
	} else {
		errorBMP = 1;
	}
	if (HMC5883L_ID() == 0x44) {
		errorHMC = 0;
	} else {
		errorHMC = 1;
	}
	if (errorMPU == 1 || errorBMP == 1 || errorHMC == 1) {
		errorSensors = 1;
	} else if (errorMPU == 0 && errorBMP == 0 && errorHMC == 0 ) {
		errorSensors = 0;
	}
}
//============ CHECK RC TRANSMITTER AND BATTERY ==============//
void peripheralCheck(){
	if (V_batt > 10) {
		errorBatt = 0;
	} else {
		errorBatt = 1;
	}
	if (channel_3 > 900 && channel_3 < 1100) {
		errorTrans = 0;
	} else {
		errorTrans = 1;
	}
	if (errorBatt == 1 || errorTrans == 1) {
		errorPeripheral = 1;
	} else if (errorBatt == 0 && errorTrans == 0) {
		errorPeripheral = 0;
	}
}
//============ CHECK BATTERY ==============//

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	// get voltage from battery
	batt_voltage = HAL_ADC_GetValue(&hadc1);
	//cal V input micro
	Vs = batt_voltage / ResDAC;
	//calculate real battery voltage using voltage divider
	V_batt = (Vs * (R1 + R2)) / R2;
	/*If continuousconversion mode is DISABLED uncomment below*/
	//HAL_ADC_Start_IT (&hadc1);
}

#endif /* INC_CHECKSYSONGROUND_H_ */
