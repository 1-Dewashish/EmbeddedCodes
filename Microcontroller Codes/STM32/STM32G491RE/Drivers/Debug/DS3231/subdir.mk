################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../DS3231/DS3231.c 

OBJS += \
./DS3231/DS3231.o 

C_DEPS += \
./DS3231/DS3231.d 


# Each subdirectory must supply rules for building sources it contributes
DS3231/%.o DS3231/%.su DS3231/%.cyclo: ../DS3231/%.c DS3231/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DNUCLEO_G491RE -DSTM32 -DSTM32G491RETx -DSTM32G4 -c -I../Inc -I"X:/STM32_Device_Driver/stm32g491re_drivers/drivers/Inc" -I"X:/STM32_Device_Driver/stm32g491re_drivers/MPU6050/Inc" -I"X:/STM32_Device_Driver/stm32g491re_drivers/DS3231" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-DS3231

clean-DS3231:
	-$(RM) ./DS3231/DS3231.cyclo ./DS3231/DS3231.d ./DS3231/DS3231.o ./DS3231/DS3231.su

.PHONY: clean-DS3231

