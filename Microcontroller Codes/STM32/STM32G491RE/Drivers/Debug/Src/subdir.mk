################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/LED_BUTTON.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/LED_BUTTON.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/LED_BUTTON.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/LED_BUTTON.o: ../Src/LED_BUTTON.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DNUCLEO_G491RE -DSTM32 -DSTM32G491RETx -DSTM32G4 -c -I../Inc -I"X:/STM32_Device_Driver/stm32g491re_drivers/MPU6050/Inc" -I"X:/STM32_Device_Driver/stm32g491re_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g -DDEBUG -DNUCLEO_G491RE -DSTM32 -DSTM32G491RETx -DSTM32G4 -c -I../Inc -I"X:/STM32_Device_Driver/stm32g491re_drivers/MPU6050/Inc" -I"X:/STM32_Device_Driver/stm32g491re_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/LED_BUTTON.cyclo ./Src/LED_BUTTON.d ./Src/LED_BUTTON.o ./Src/LED_BUTTON.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su

.PHONY: clean-Src

