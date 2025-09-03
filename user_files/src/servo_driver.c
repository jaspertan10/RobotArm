
#include "servo_driver.h"

//HAL_StatusTypeDef I2C_master_write_slave(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t controlRegAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//{
//
//}

static I2C_HandleTypeDef *servo_i2c_handle = NULL;

void servo_driver_init(I2C_HandleTypeDef *hi2c)
{
	servo_i2c_handle = hi2c;


	// Servo driver must be in sleep prior to configuring pre scale
	servo_driver_sleep();

	/* Set the pre scale to make PWM outputs to motors 50Hz */
	servo_driver_set_prescale(SERVO_MOTOR_FREQUENCY_HZ);

	// Wakeup servo driver, requires 500us max for oscillator to be up
	servo_driver_wakeup();
	HAL_Delay(1);

}

void servo_set_pwm(servo_channels_t channel, servo_pwm_preset_t servo_angle)
{
	uint8_t temp_pwm_high;
	uint8_t temp_pwm_low;


	//todo: improve this to remove if statement when configuring pwm registers
	if (channel == SERVO_0)
	{
		// Set PWM pulse width
		temp_pwm_high = ((uint16_t)servo_angle >> 8);
		temp_pwm_low = (uint8_t)servo_angle;
		HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_PWM0_OFF_H, 1, &temp_pwm_high, 1, SERVO_DRIVER_I2C_TIMEOUT);
		HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_PWM0_OFF_L, 1, &temp_pwm_low, 1, SERVO_DRIVER_I2C_TIMEOUT);


		temp_pwm_high = 0;
		temp_pwm_low = 0;
		HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_PWM0_ON_H, 1, &temp_pwm_high, 1, SERVO_DRIVER_I2C_TIMEOUT);
		HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_PWM0_ON_L, 1, &temp_pwm_low, 1, SERVO_DRIVER_I2C_TIMEOUT);
	}
}

/**
  * @brief  Sets servo driver output frequency in hertz (hz)
  * @param  output_freq Output frequency in hz

  */
void servo_driver_set_prescale(uint16_t output_freq)
{
	uint8_t write_reg_temp;

	write_reg_temp = ((SERVO_DRIVER_OSCILLATOR_FREQ) / (SERVO_DRIVER_PWM_STEPS * output_freq)) - 1;
	HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_PRE_SCALE_REG_ADDR, 1, &write_reg_temp, 1, SERVO_DRIVER_I2C_TIMEOUT);
}

void servo_driver_sleep()
{
	uint8_t write_reg_temp;
	uint8_t read_reg_temp;

	//Read Mode1 register
	HAL_I2C_Mem_Read(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_MODE1_REG_ADDR, 1, &read_reg_temp, 1, SERVO_DRIVER_I2C_TIMEOUT);

	// Set the sleep bit in MODE1 register
	write_reg_temp = read_reg_temp;
	write_reg_temp |= (1 << 4);
	HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_MODE1_REG_ADDR, 1, &write_reg_temp, 1, SERVO_DRIVER_I2C_TIMEOUT);
}

void servo_driver_wakeup()
{
	uint8_t write_reg_temp;
	uint8_t read_reg_temp;

	//Read Mode1 register
	HAL_I2C_Mem_Read(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_MODE1_REG_ADDR, 1, &read_reg_temp, 1, SERVO_DRIVER_I2C_TIMEOUT);

	// Set the sleep bit in MODE1 register
	write_reg_temp = read_reg_temp;
	write_reg_temp &= ~(1 << 4);
	HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, SERVO_DRIVER_MODE1_REG_ADDR, 1, &write_reg_temp, 1, SERVO_DRIVER_I2C_TIMEOUT);
}
