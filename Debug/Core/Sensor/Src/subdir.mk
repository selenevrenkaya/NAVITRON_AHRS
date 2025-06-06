################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sensor/Src/bno055.c 

OBJS += \
./Core/Sensor/Src/bno055.o 

C_DEPS += \
./Core/Sensor/Src/bno055.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sensor/Src/%.o Core/Sensor/Src/%.su Core/Sensor/Src/%.cyclo: ../Core/Sensor/Src/%.c Core/Sensor/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Comm" -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Sensor/Inc" -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Sensor/Src" -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Filter" -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Sensor" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sensor-2f-Src

clean-Core-2f-Sensor-2f-Src:
	-$(RM) ./Core/Sensor/Src/bno055.cyclo ./Core/Sensor/Src/bno055.d ./Core/Sensor/Src/bno055.o ./Core/Sensor/Src/bno055.su

.PHONY: clean-Core-2f-Sensor-2f-Src

