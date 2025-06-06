################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Sensor/bno055.c \
../Core/Sensor/mpu9255.c \
../Core/Sensor/ms5611.c \
../Core/Sensor/neo_m8n.c 

OBJS += \
./Core/Sensor/bno055.o \
./Core/Sensor/mpu9255.o \
./Core/Sensor/ms5611.o \
./Core/Sensor/neo_m8n.o 

C_DEPS += \
./Core/Sensor/bno055.d \
./Core/Sensor/mpu9255.d \
./Core/Sensor/ms5611.d \
./Core/Sensor/neo_m8n.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Sensor/%.o Core/Sensor/%.su Core/Sensor/%.cyclo: ../Core/Sensor/%.c Core/Sensor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Sensor" -I"C:/Users/LENOVO/STM32CubeIDE/workspace_1.13.2/NAVITRON_AHRS/Core/Comm" -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Sensor

clean-Core-2f-Sensor:
	-$(RM) ./Core/Sensor/bno055.cyclo ./Core/Sensor/bno055.d ./Core/Sensor/bno055.o ./Core/Sensor/bno055.su ./Core/Sensor/mpu9255.cyclo ./Core/Sensor/mpu9255.d ./Core/Sensor/mpu9255.o ./Core/Sensor/mpu9255.su ./Core/Sensor/ms5611.cyclo ./Core/Sensor/ms5611.d ./Core/Sensor/ms5611.o ./Core/Sensor/ms5611.su ./Core/Sensor/neo_m8n.cyclo ./Core/Sensor/neo_m8n.d ./Core/Sensor/neo_m8n.o ./Core/Sensor/neo_m8n.su

.PHONY: clean-Core-2f-Sensor

