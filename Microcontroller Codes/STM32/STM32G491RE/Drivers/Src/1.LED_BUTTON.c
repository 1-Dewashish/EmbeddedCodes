/**
 ******************************************************************************
 * @file           : LED_Button
 * @author         : Dewashish Deshmukh
 * @brief          : Toggle the LED after BUTTON is pressed.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32g491re.h"
#include "stm32g491re_GPIO_DRIVER.h"



#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

// Simple for loop delay
void delay(void){
	for (uint32_t i = 0; i<500000;i++);
}


int main(void)
{
	/*Created a GPIO Handle for on board LED connected to GPIOA 5*/
	GPIO_Handle_t led;
	led.pGPIO = GPIOA;
	led.GPIO_Pinconfig.GPIO_PinNumber = 5;
	led.GPIO_Pinconfig.GPIO_PinMode = OUTPUT;
	led.GPIO_Pinconfig.GPIO_PinOPType = PUSH_PULL;
	led.GPIO_Pinconfig.GPIO_PinSpeed = MEDIUM_SPEED;
	led.GPIO_Pinconfig.GPIO_PinPuPdControl= NO_PUPD;

	/*Created a GPIO Handle for on board USER_BUTTON connected to GPIOC 13*/
	GPIO_Handle_t button;
	button.pGPIO = GPIOC;
	button.GPIO_Pinconfig.GPIO_PinNumber =13;
	button.GPIO_Pinconfig.GPIO_PinMode = INPUT;
	button.GPIO_Pinconfig.GPIO_PinSpeed = HIGH_SPEED;
	button.GPIO_Pinconfig.GPIO_PinPuPdControl = PD;

	/*Enabling GPIOs clock*/
	GPIO_PCLKControl(GPIOA,ENABLE);
	GPIO_PCLKControl(GPIOC,ENABLE);

	/*Passing GPIO handle*/
	GPIO_INIT(&led);
	GPIO_INIT(&button);


	while(1){
		/*This while loop continuously checks the button status and if button is pressed LED is toggled*/

		if (GPIO_ReadFromInputPin(GPIOC, 13)){
			/*added delay for suppressing de-bouncing*/
 			delay();
			GPIO_ToggleOutputPin(GPIOA,5);
		}
	}


	return 0;
}






