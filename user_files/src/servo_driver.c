
#include "servo_driver.h"
#include "stdint.h"

typedef struct
{
	uint8_t on_l;
	uint8_t on_h;
	uint8_t off_l;
	uint8_t off_h;

} servo_channel_registers_t;

static const servo_channel_registers_t servo_register_map[SERVO_NUM_OF_SERVOS] = {
	{0x06, 0x07, 0x08, 0x09},	//Channel 0
	{0x0A, 0x0B, 0x0C, 0x0D},	//Channel 1
	{0x0E, 0x0F, 0x10, 0x11},	//Channel 2
	{0x12, 0x13, 0x14, 0x15},	//Channel 3
	{0x16, 0x17, 0x18, 0x19},	//Channel 4
	{0x1A, 0x1B, 0x1C, 0x1D},	//Channel 5
};



servo_config_t servos[SERVO_NUM_OF_SERVOS] = {
	// Initialize servos to all be at mid point
	{
			.channel = CH0_LEFT_SHOULDER,
			.min_tick = SERVO_GENERAL_0_DEGREES_TICK,
			.max_tick = SERVO_GENERAL_180_DEGREES_TICK,
			.mid_point_tick = SERVO_LEFT_SHOULDER_MIDPOINT_TICK,
			.current_tick = SERVO_LEFT_SHOULDER_MIDPOINT_TICK,
	},

	{
			.channel = CH1_RIGHT_SHOULDER,
			.min_tick = SERVO_GENERAL_0_DEGREES_TICK,
			.max_tick = SERVO_GENERAL_180_DEGREES_TICK,
			.mid_point_tick = SERVO_GENERAL_90_DEGREES_TICK,
			.current_tick = SERVO_GENERAL_90_DEGREES_TICK
	},

	{
			.channel = CH2_ELBOW,
			.min_tick = SERVO_GENERAL_0_DEGREES_TICK,
			.max_tick = SERVO_GENERAL_180_DEGREES_TICK,
			.mid_point_tick = SERVO_GENERAL_90_DEGREES_TICK,
			.current_tick = SERVO_GENERAL_90_DEGREES_TICK
	},

	{
			.channel = CH3_WRIST,
			.min_tick = SERVO_GENERAL_0_DEGREES_TICK,
			.max_tick = SERVO_GENERAL_180_DEGREES_TICK,
			.mid_point_tick = SERVO_GENERAL_90_DEGREES_TICK,
			.current_tick = SERVO_GENERAL_90_DEGREES_TICK
	},

	{
			.channel = CH4_GRIPPER_ROTATOR,
			.min_tick = SERVO_GENERAL_0_DEGREES_TICK,
			.max_tick = SERVO_GENERAL_180_DEGREES_TICK,
			.mid_point_tick = SERVO_GENERAL_90_DEGREES_TICK,
			.current_tick = SERVO_GENERAL_90_DEGREES_TICK
	},

	{
			.channel = CH5_GRIPPER,
			.min_tick = SERVO_GRIPPER_CLOSE_TICK,
			.max_tick = SERVO_GRIPPER_OPEN_TICK,
			.mid_point_tick = SERVO_GENERAL_90_DEGREES_TICK,
			.current_tick = SERVO_GRIPPER_OPEN_TICK
	},
};

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

/* 	 PCA9685 PWM channels have 4096 steps.
	 on_l and on_h set when the signal goes HIGH; off_l and off_h set when it goes LOW.
	 For our servos, we always start HIGH immediately (on_l = on_h = 0),
	 and off_l/off_h are set according to servo_tick to control the servo position. */
void servo_set_pwm(servo_config_t servo, uint16_t servo_tick)
{
	uint8_t pwm_low_step = 0;
	uint8_t pwm_high_step = 0;

	const servo_channel_registers_t servo_regs = servo_register_map[servo.channel];

	//Channel PWM set to ON (HIGH) right immediately (i.e. on_l = on_h = 0)
	HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, servo_regs.on_l, 1, &pwm_low_step, 1, SERVO_DRIVER_I2C_TIMEOUT);
	HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, servo_regs.on_h, 1, &pwm_high_step, 1, SERVO_DRIVER_I2C_TIMEOUT);

	// Set PWM pulse width based on servo_tick, in general, servo_tick = 80 = 0 degrees, servo_tick = 560 = 180 degrees.
	pwm_low_step = (uint8_t)servo_tick;
	pwm_high_step = (uint8_t)(servo_tick >> 8);
	HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, servo_regs.off_l, 1, &pwm_low_step, 1, SERVO_DRIVER_I2C_TIMEOUT);
	HAL_I2C_Mem_Write(servo_i2c_handle, SERVO_DRIVER_I2C_ADDR << 1, servo_regs.off_h, 1, &pwm_high_step, 1, SERVO_DRIVER_I2C_TIMEOUT);
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
