/*
 * ReadRC6H.h
 *
 *  Created on: Jan 29, 2021
 *      Author: LeeSeng
 */

#ifndef READRC6H_H_
#define READRC6H_H_

int32_t channel_1_start, channel_1;
int32_t channel_2_start, channel_2;
int32_t channel_3_start, channel_3;
int32_t channel_4_start, channel_4;
int32_t channel_5_start, channel_5;
int32_t channel_6_start, channel_6;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	  	//This function is called when channel 1 is captured.
	    if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == GPIO_PIN_SET) 		//If the receiver channel 1 input pulse on B6 is high.
	    {
	      channel_1_start = TIM3->CCR1;                 				//Record the start time of the pulse.
	      TIM3->CCER |= TIM_CCER_CC1P;                					//Change the input capture mode to the falling edge of the pulse.
	    }
	    else if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6) == GPIO_PIN_RESET) 	//If the receiver channel 1 input pulse on B6 is low.
	    {
	      channel_1 = TIM3->CCR1 - channel_1_start;     				//Calculate the total pulse time.
	      if (channel_1 < 0)channel_1 += 0xFFFF;               			//If the timer has rolled over a correction is needed.
	      TIM3->CCER &= ~TIM_CCER_CC1P;               					//Change the input capture mode to the rising edge of the pulse.
	    }

	    //This function is called when channel 2 is captured.
	    if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == GPIO_PIN_SET) 		//If the receiver channel 2 input pulse on B7 is high.
	    {
	      channel_2_start = TIM3->CCR2;                 				//Record the start time of the pulse.
	      TIM3->CCER |= TIM_CCER_CC2P;                					//Change the input capture mode to the falling edge of the pulse.
	    }
	    else if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7) == GPIO_PIN_RESET) 	//If the receiver channel 2 input pulse on B7 is low.
	    {
	      channel_2 = TIM3->CCR2 - channel_2_start;     				//Calculate the total pulse time.
	      if (channel_2 < 0)channel_2 += 0xFFFF;               			//If the timer has rolled over a correction is needed.
	      TIM3->CCER &= ~TIM_CCER_CC2P;               					//Change the input capture mode to the rising edge of the pulse.
	    }

	    //This function is called when channel 3 is captured.
	    if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_SET) 		//If the receiver channel 3 input pulse on B8 is high.
	    {
	      channel_3_start = TIM3->CCR3;                 				//Record the start time of the pulse.
	      TIM3->CCER |= TIM_CCER_CC3P;                					//Change the input capture mode to the falling edge of the pulse.
	    }
	    else if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_RESET) 	//If the receiver channel 3 input pulse on B8 is low.
	    {
	      channel_3 = TIM3->CCR3 - channel_3_start;     				//Calculate the total pulse time.
	      if (channel_3 < 0)channel_3 += 0xFFFF;               			//If the timer has rolled over a correction is needed.
	      TIM3->CCER &= ~TIM_CCER_CC3P;               					//Change the input capture mode to the rising edge of the pulse.
	    }

	    //This function is called when channel 4 is captured.
	    if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == GPIO_PIN_SET) 		//If the receiver channel 4 input pulse on B9 is high.
	    {
	      channel_4_start = TIM3->CCR4;                 				//Record the start time of the pulse.
	      TIM3->CCER |= TIM_CCER_CC4P;                					//Change the input capture mode to the falling edge of the pulse.
	    }
	    else if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == GPIO_PIN_RESET) 	//If the receiver channel 4 input pulse on B9 is low.
	    {
	      channel_4 = TIM3->CCR4 - channel_4_start;     				//Calculate the total pulse time.
	      if (channel_4 < 0)channel_4 += 0xFFFF;               			//If the timer has rolled over a correction is needed.
	      TIM3->CCER &= ~TIM_CCER_CC4P;               					//Change the input capture mode to the rising edge of the pulse.
	    }

	    //This function is called when channel 5 is captured.
	    if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_SET) 		//If the receiver channel 5 input pulse on A0 is high.
	    {
	      channel_5_start = TIM4->CCR1;                 				//Record the start time of the pulse.
	      TIM4->CCER |= TIM_CCER_CC1P;                					//Change the input capture mode to the falling edge of the pulse.
	    }
	    else if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_6) == GPIO_PIN_RESET) 	//If the receiver channel 5 input pulse on A0 is low.
	    {
	      channel_5 = TIM4->CCR1 - channel_5_start;     				//Calculate the total pulse time.
	      if (channel_5 < 0)channel_5 += 0xFFFF;               			//If the timer has rolled over a correction is needed.
	      TIM4->CCER &= ~TIM_CCER_CC1P;               					//Change the input capture mode to the rising edge of the pulse.
	    }

	    //This function is called when channel 6 is captured.
	    if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7) == GPIO_PIN_SET) 		//If the receiver channel 6 input pulse on A1 is high.
	    {
	      channel_6_start = TIM4->CCR2;                 				//Record the start time of the pulse.
	      TIM4->CCER |= TIM_CCER_CC2P;                					//Change the input capture mode to the falling edge of the pulse.
	    }
	    else if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7) == GPIO_PIN_RESET) 	//If the receiver channel 6 input pulse on A1 is low.
	    {
	      channel_6 = TIM4->CCR2 - channel_6_start;     				//Calculate the total pulse time.
	      if (channel_6 < 0)channel_6 += 0xFFFF;               			//If the timer has rolled over a correction is needed.
	      TIM4->CCER &= ~TIM_CCER_CC2P;               					//Change the input capture mode to the rising edge of the pulse.
	    }

}

#endif /* READRC6H_H_ */
