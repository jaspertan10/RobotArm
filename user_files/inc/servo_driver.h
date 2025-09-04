

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "stm32f4xx_hal.h"

/*
 * Servo driver used: PCA9685
 * Servo motors used: MG9996R and MG9950
 */

/* Servo Driver Macros */
#define SERVO_DRIVER_I2C_ADDR					0x40
#define SERVO_DRIVER_OSCILLATOR_FREQ 			25000000
#define SERVO_DRIVER_PWM_STEPS					4096
#define SERVO_DRIVER_I2C_TIMEOUT				1000
#define SERVO_DRIVER_ON							1
#define SERVO_DRIVER_OFF						0
#define SERVO_NUM_OF_SERVOS						6
#define SERVO_CHANNEL_0							0
#define SERVO_CHANNEL_1							1
#define SERVO_CHANNEL_2							2
#define SERVO_CHANNEL_3							3
#define SERVO_CHANNEL_4							4
#define SERVO_CHANNEL_5							5
#define SERVO_LEFT_SHOULDER_MIDPOINT_TICK		270
#define SERVO_GRIPPER_OPEN_TICK					540
#define SERVO_GRIPPER_CLOSE_TICK				120

/* Servo Driver Control Registers */
#define SERVO_DRIVER_MODE1_REG_ADDR				0x00
#define SERVO_DRIVER_PRE_SCALE_REG_ADDR			0xFE


/* Servo Motor */
#define SERVO_MOTOR_FREQUENCY_HZ				50
#define SERVO_GENERAL_0_DEGREES_TICK			80
#define SERVO_GENERAL_90_DEGREES_TICK			280
#define SERVO_GENERAL_180_DEGREES_TICK			560

typedef enum
{
	CH0_LEFT_SHOULDER 		= 0,
	CH1_RIGHT_SHOULDER 		= 1,
	CH2_ELBOW 				= 2,
	CH3_WRIST 				= 3,
	CH4_GRIPPER_ROTATOR		= 4,
	CH5_GRIPPER 			= 5
} servo_part_channel_num_t;

typedef struct
{
	//Servo Channel Information
	const servo_part_channel_num_t channel;			// Channel numbers 0-15

	// Servo range (in ticks)
	const uint16_t min_tick;		// Servo tick num for 0 degrees
	const uint16_t max_tick;		// Servo tick num for 180 degrees
	const uint16_t mid_point_tick;	// Servo tick num for mid point

	// Servo position
	uint16_t current_tick;			// Current servo tick, must be between min_tick and max_tick

} servo_config_t;


extern servo_config_t servos[SERVO_NUM_OF_SERVOS];



/* Function Declarations */
void servo_driver_init(I2C_HandleTypeDef *hi2c);
void servo_set_pwm(servo_config_t servo, uint16_t servo_tick);
void servo_driver_set_prescale(uint16_t output_freq);
void servo_driver_sleep();
void servo_driver_wakeup();


#endif //SERVO_DRIVER_H
