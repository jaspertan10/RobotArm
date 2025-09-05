
/*
 * Stepper motor: Nema17
 * Stepper motor driver: A4988
 */

#ifndef STEPPER_DRIVER_H
#define STEPPER_DRIVER_H


/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/
typedef enum
{
	STEPPER_COUNTER_CLOCKWISE_DIR 		= 0,
	STEPPER_CLOCKWISE_DIR 				= 1,

} stepper_direction_t;


/* Private define ------------------------------------------------------------*/
#define STEPS_PER_REVOLUTION			200
#define STEPS_AMOUNT					25


/* Private variables ---------------------------------------------------------*/
static int stepper_step_count = 0;


/* Private function prototypes -----------------------------------------------*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);



void stepper_set_direction(stepper_direction_t direction);



#endif
