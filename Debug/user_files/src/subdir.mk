################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../user_files/src/servo_driver.c 

OBJS += \
./user_files/src/servo_driver.o 

C_DEPS += \
./user_files/src/servo_driver.d 


# Each subdirectory must supply rules for building sources it contributes
user_files/src/%.o user_files/src/%.su user_files/src/%.cyclo: ../user_files/src/%.c user_files/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../user_files/inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-user_files-2f-src

clean-user_files-2f-src:
	-$(RM) ./user_files/src/servo_driver.cyclo ./user_files/src/servo_driver.d ./user_files/src/servo_driver.o ./user_files/src/servo_driver.su

.PHONY: clean-user_files-2f-src

