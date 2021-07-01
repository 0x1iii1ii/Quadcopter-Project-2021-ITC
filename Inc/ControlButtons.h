/*
 * ControlButtons.h
 *
 *  Created on: Jan 31, 2021
 *      Author: LiiSeng
 */

#ifndef CONTROLBUTTONS_H_
#define CONTROLBUTTONS_H_

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;

#include "buzzerSound.h"
#include "checkSysOnGround.h"

extern uint8_t errorSensors, errorPeripheral;

bool armButton = false;
uint8_t start = 0;
bool startError = false;
bool startGood = false;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (errorSensors == 0 && errorPeripheral == 0) {
		//can arm if system good
		//start  timer arm button
		HAL_TIM_Base_Start_IT(&htim1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim9) {
		if (errorSensors == 1 || errorPeripheral == 1) {
			//blink red if error
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
			startError = true;
		} else if (errorSensors == 0 && errorPeripheral == 0) {
			//blink green if good
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
			startGood = true;
		}
	}
	if (htim == &htim10) {
		if (start == 2) {
			//blink blue after start at second phase
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		}
	}
	//after 4 second, check the arm button
	if (htim == &htim1) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET) {
			armButton = true;
			start = 1;
			HAL_TIM_Base_Stop_IT(&htim1);
		}
	}
}

#endif /* CONTROLBUTTONS_H_ */
