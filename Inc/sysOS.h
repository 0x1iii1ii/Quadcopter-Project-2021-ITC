/*
 * sysOS.h
 *
 *  Created on: Jan 29, 2021
 *      Author: LeeSeng
 */

#ifndef SYSOS_H_
#define SYSOS_H_

#include "checkSysOnGround.h"
#include "ControlButtons.h"

void startCheckSound()
{
	if (errorSensors == 1 || errorPeripheral == 1) {
		errorSound();
	} else if (errorSensors == 0 || errorPeripheral == 0) {
		startSound();
	}
}
void armDisarm1() {
	if (armButton) {
		//set blue on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		armedSound1();
		armButton = 0;
	}
}
void armDisarm2() {
	if (start) {
		if (channel_3 < 1100 && channel_4 > 1900) {
			start = 2;
			armedSound2();
			//start blink blue
			HAL_TIM_Base_Start_IT(&htim10);
		}
		if (channel_3 < 1100 && channel_4 < 1100) {
			errorSound();
			//stop blink blue
			HAL_TIM_Base_Stop_IT(&htim10);
			//set blue on back
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			start = 1;
		}
	}
}

#endif /* SYSOS_H_ */
