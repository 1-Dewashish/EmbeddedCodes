################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MPU6050/Src/MPU6050.c 

OBJS += \
./MPU6050/Src/MPU6050.o 

C_DEPS += \
./MPU6050/Src/MPU6050.d 


# Each subdirectory must supply rules for building sources it contributes
MPU6050/Src/%.o MPU6050/Src/%.su MPU6050/Src/%.cyclo: ../MPU6050/Src/%.c MPU6050/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DNUCLEO_G491RE -DSTM32 -DSTM32G491RETx -DSTM32G4 -c -I../Inc -I"X:/STM32_Device_Driver/stm32g491re_drivers/drivers/Inc" -I"X:/STM32_Device_Driver/stm32g491re_drivers/MPU6050/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MPU6050-2f-Src

clean-MPU6050-2f-Src:
	-$(RM) ./MPU6050/Src/MPU6050.cyclo ./MPU6050/Src/MPU6050.d ./MPU6050/Src/MPU6050.o ./MPU6050/Src/MPU6050.su

.PHONY: clean-MPU6050-2f-Src

