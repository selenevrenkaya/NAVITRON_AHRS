################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Comm/circ_buffer.c 

OBJS += \
./Core/Comm/circ_buffer.o 

C_DEPS += \
./Core/Comm/circ_buffer.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Comm/%.o Core/Comm/%.su Core/Comm/%.cyclo: ../Core/Comm/%.c Core/Comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Sensor" -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Comm" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Comm

clean-Core-2f-Comm:
	-$(RM) ./Core/Comm/circ_buffer.cyclo ./Core/Comm/circ_buffer.d ./Core/Comm/circ_buffer.o ./Core/Comm/circ_buffer.su

.PHONY: clean-Core-2f-Comm

