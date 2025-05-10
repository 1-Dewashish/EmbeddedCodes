/*
 * stm32g491re_gpio_driver.h
 *
 *  Created on: Feb 9, 2025
 *      Author: ASUS
 */

#ifndef INC_STM32G491RE_GPIO_DRIVER_H_
#define INC_STM32G491RE_GPIO_DRIVER_H_

#include "stm32g491re.h"


/*****GPIO MODER*****/
#define INPUT							0
#define OUTPUT							1
#define ALT_Function					2
#define Analog							3
#define RT								4
#define FT								5
#define RT_FT							6

/*****GPIO OUTPUT TYPE*****/
#define PUSH_PULL						RESET
#define OPEN_DRAIN						SET

/*****GPIO Output Speed*****/
#define LOW_SPEED						0
#define MEDIUM_SPEED					1
#define HIGH_SPEED						2
#define VERY_HIGH_SPEED					3

/*****GPIO PUPDR*****/
#define NO_PUPD							0
#define PU								1
#define PD								2



typedef struct{

	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


/****GPIO Handle structure****/
typedef struct{
	GPIO_RegDef_t* pGPIO ;
	GPIO_PinConfig_t  GPIO_Pinconfig;
}GPIO_Handle_t;







/********GPIO CLK enble*********/
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIO, uint8_t ENorDI );


/****GPIO INIT/DINIT****/
void GPIO_INIT(GPIO_Handle_t *pGPIOHandle);
void GPIO_DENIT(GPIO_RegDef_t *pGPIO);



/****GPIO READ/WRITE TO PORT/PIN****/
uint8_t	 GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber,uint8_t value);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber);


/****GPIO INTERRUPT HANDLING****/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQHandling(uint8_t PinNumber);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

#endif /* INC_STM32G491RE_GPIO_DRIVER_H_ */
