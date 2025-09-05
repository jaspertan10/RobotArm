

#include "main.h"
#include "stepper_driver.h"


void stepper_set_direction(stepper_direction_t direction)
{
	if (direction == STEPPER_COUNTER_CLOCKWISE_DIR)
	{
		HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_RESET);
	}
	else if (direction == STEPPER_CLOCKWISE_DIR)
	{
		HAL_GPIO_WritePin(DIR_PIN_GPIO_Port, DIR_PIN_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
   if (htim->Instance == TIM10)
   {
	   stepper_step_count++;

	   // Deceleration ramp for last 20 steps
	   if (stepper_step_count >= STEPS_AMOUNT * 2) //Multiply by 2 as stepper_step_count increments on rising and falling edge of STEP GPIO output
	   {
		   HAL_TIM_OC_Stop_IT(htim, TIM_CHANNEL_1);
		   stepper_step_count = 0;
	   }

   }
}
