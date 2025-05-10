################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32g491re_gpio_driver.c \
../drivers/Src/stm32g491re_i2c_driver.c \
../drivers/Src/stm32g491re_simple_timer_driver.c \
../drivers/Src/stm32g491re_spi_driver.c \
../drivers/Src/stm32g491re_uart.c 

OBJS += \
./drivers/Src/stm32g491re_gpio_driver.o \
./drivers/Src/stm32g491re_i2c_driver.o \
./drivers/Src/stm32g491re_simple_timer_driver.o \
./drivers/Src/stm32g491re_spi_driver.o \
./drivers/Src/stm32g491re_uart.o 

C_DEPS += \
./drivers/Src/stm32g491re_gpio_driver.d \
./drivers/Src/stm32g491re_i2c_driver.d \
./drivers/Src/stm32g491re_simple_timer_driver.d \
./drivers/Src/stm32g491re_spi_driver.d \
./drivers/Src/stm32g491re_uart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_G491RE -DSTM32 -DSTM32G491RETx -DSTM32G4 -c -I../Inc -I"X:/STM32_Device_Driver/stm32g491re_drivers/drivers/Inc" -I"X:/STM32_Device_Driver/stm32g491re_drivers/MPU6050/Inc" -I"X:/STM32_Device_Driver/stm32g491re_drivers/DS3231" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32g491re_gpio_driver.cyclo ./drivers/Src/stm32g491re_gpio_driver.d ./drivers/Src/stm32g491re_gpio_driver.o ./drivers/Src/stm32g491re_gpio_driver.su ./drivers/Src/stm32g491re_i2c_driver.cyclo ./drivers/Src/stm32g491re_i2c_driver.d ./drivers/Src/stm32g491re_i2c_driver.o ./drivers/Src/stm32g491re_i2c_driver.su ./drivers/Src/stm32g491re_simple_timer_driver.cyclo ./drivers/Src/stm32g491re_simple_timer_driver.d ./drivers/Src/stm32g491re_simple_timer_driver.o ./drivers/Src/stm32g491re_simple_timer_driver.su ./drivers/Src/stm32g491re_spi_driver.cyclo ./drivers/Src/stm32g491re_spi_driver.d ./drivers/Src/stm32g491re_spi_driver.o ./drivers/Src/stm32g491re_spi_driver.su ./drivers/Src/stm32g491re_uart.cyclo ./drivers/Src/stm32g491re_uart.d ./drivers/Src/stm32g491re_uart.o ./drivers/Src/stm32g491re_uart.su

.PHONY: clean-drivers-2f-Src

