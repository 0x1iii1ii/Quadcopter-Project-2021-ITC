/*
 * customFunction.h
 *
 *  Created on: Feb 19, 2021
 *      Author: LeeSeng
 */

#ifndef CUSTOMFUNCTION_H_
#define CUSTOMFUNCTION_H_

//map function
//map(var to map, in_min, in_max, out_min, out_max)
float map(float val, float I_Min, float I_Max, float O_Min, float O_Max) {
	return (((val - I_Min) * ((O_Max - O_Min) / (I_Max - I_Min))) + O_Min);
}
//led RGB function
//RGB_color(htim, red, green, blue)
void RGB_color(TIM_HandleTypeDef *htim, int red_light_value,
		int green_light_value, int blue_light_value) {
	htim->Instance->CCR1 = red_light_value;
	htim->Instance->CCR2 = green_light_value;
	htim->Instance->CCR3 = blue_light_value;
}
//buzzer tone function
//toneStart();
//tone(pin, frequency)
//toneUntil(htim, ch, frequency, duration)
//toneStop();
void toneUntil(TIM_HandleTypeDef *htim, uint32_t Channel, uint16_t valueFreq,
		uint16_t during) {

	uint16_t count = HAL_GetTick(); 				//save start time
	uint16_t autoReload = htim->Init.Period; 		// get ARR
	uint16_t prescaler;
	uint32_t clockSource = 100000000; 				//clock source 84Mhz
	HAL_TIM_PWM_Start(htim, Channel); 				//start timer
	uint16_t halfDuty = autoReload / 2; 			// 50% duty cycle
	htim->Instance->CCR1 = halfDuty;
	while (HAL_GetTick() - count <= during) { //check if equal to input duration
		//calculate prescaler from input frequency
		prescaler = clockSource / ((autoReload) * valueFreq);
		htim->Instance->PSC = prescaler;
	}
	HAL_TIM_PWM_Stop(htim, Channel); 				// if so stop timer
	count = HAL_GetTick(); 							// update count

}
void tone(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t valueFreq) {

	//same with toneUntil but no limit time
	uint16_t autoReload = htim->Init.Period;
	uint16_t prescaler;
	uint32_t clockSource = 84000000;
	HAL_TIM_PWM_Start(htim, Channel);
	uint16_t halfDuty = autoReload / 2;
	htim->Instance->CCR1 = halfDuty;
	prescaler = clockSource / ((autoReload) * valueFreq);
	htim->Instance->PSC = prescaler;

}
void toneStop(TIM_HandleTypeDef *htim, uint32_t Channel) {
	HAL_TIM_PWM_Stop(htim, Channel);
}
#endif /* INC_CUSTOMFUNCTION_H_ */
