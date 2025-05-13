/*
 * @file           : I2C Bus Scan
 * @author         : Dewashish Deshmukh
 * @brief          : Gives the list of all address of all I2C device available.
 */


#include <stdio.h>
#include <stdint.h>
#include <string.h>


#include "stm32g491re.h"
#include "stm32g491re_GPIO_DRIVER.h"
#include "stm32g491re_I2C_driver.h"



void I2C2_GPIOInit(void){

	GPIO_Handle_t I2CPins;
	I2CPins.pGPIO=GPIOC;
	I2CPins.GPIO_Pinconfig.GPIO_PinMode = ALT_Function;
	I2CPins.GPIO_Pinconfig.GPIO_PinAltFunMode = 8;
	I2CPins.GPIO_Pinconfig.GPIO_PinOPType = OPEN_DRAIN;
	I2CPins.GPIO_Pinconfig.GPIO_PinPuPdControl = NO_PUPD;
	I2CPins.GPIO_Pinconfig.GPIO_PinSpeed = VERY_HIGH_SPEED;

	//  PC9 = SDA PC8 = SCL
	//SCL
	I2CPins.GPIO_Pinconfig.GPIO_PinNumber = 8;
	GPIO_INIT(&I2CPins);

	//SDA
	I2CPins.GPIO_Pinconfig.GPIO_PinNumber = 9;
	GPIO_INIT(&I2CPins);



}
void I2Cx_INIT(void){

	I2C_Handle_t  i2cHandle;
	i2cHandle.pI2C = I2C3;
	i2cHandle.I2CConfig.I2CCKL = I2Cx_CLK_PCLK;
	i2cHandle.I2CConfig.I2C_SPEED = 300000;
	i2cHandle.I2CConfig.I2C_HOLD_TIME =1000 ;
	i2cHandle.I2CConfig.I2C_SETUP_TIME =1000;
	i2cHandle.I2CConfig.I2C_SCL_HIGH_PERIOD =5000;
	i2cHandle.I2CConfig.I2C_SCL_LOW_PERIOD =5000;
	I2C_INIT(&i2cHandle);

}
int main(void){

	/*Initialize the GPIOs for I2C Communication */
	I2C2_GPIOInit();

	/*Initialize the  I2C Peripheral */
	I2Cx_INIT();

	/* Prints the available I2C Devices*/
	I2CScanDevices(I2C3);

	return 0;
}


