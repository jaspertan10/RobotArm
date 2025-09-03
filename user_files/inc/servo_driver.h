

#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

#include "main.h"

#define ONE_BYTE								1
#define TWO_BYTES								2

typedef enum
{
	SERVO_PWM_0_DEGREES = 205,
	SERVO_PWM_90_DEGREES = 307,
	SERVO_PWM_180_DEGREES = 410

} servo_pwm_preset_t;

typedef enum
{
	SERVO_0,
	SERVO_1,
	SERVO_2,
	SERVO_3,
	SERVO_4,
	SERVO_5,
	SERVO_6,
	SERVO_7
} servo_channels_t;

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

/* Servo Driver Control Registers */
#define SERVO_DRIVER_MODE1_REG_ADDR				0x00
#define SERVO_DRIVER_PRE_SCALE_REG_ADDR			0xFE
#define SERVO_DRIVER_PWM0_ON_L					0x06
#define SERVO_DRIVER_PWM0_ON_H					0x07
#define SERVO_DRIVER_PWM0_OFF_L					0x08
#define SERVO_DRIVER_PWM0_OFF_H					0x09

/* Servo Motor */
#define SERVO_MOTOR_FREQUENCY_HZ				50



/* Function Declarations */
void servo_driver_init(I2C_HandleTypeDef *hi2c);
void servo_set_pwm(servo_channels_t channel, servo_pwm_preset_t servo_angle);
void servo_driver_set_prescale(uint16_t output_freq);
void servo_driver_sleep();
void servo_driver_wakeup();


#endif //SERVO_DRIVER_H
