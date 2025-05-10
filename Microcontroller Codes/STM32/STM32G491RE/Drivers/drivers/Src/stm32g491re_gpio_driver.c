/*
 * stm32g491re_gpio.c
 *
 *  Created on: Feb 9, 2025
 *      Author: ASUS
 */


#include "stm32g491re.h"
#include "stm32g491re_gpio_driver.h"


/********GPIO CLK enable*********/
void GPIO_PCLKControl(GPIO_RegDef_t *pGPIO, uint8_t ENorDI ){

	if (ENorDI == ENABLE){
		if (pGPIO == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIO == GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIO == GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIO == GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIO == GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIO == GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIO == GPIOG){
			GPIOG_PCLK_EN();
		}
	} else if(ENorDI == DISABLE){
		if (pGPIO == GPIOA){
			GPIOA_PCLK_DI();
		}else if (pGPIO == GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIO == GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIO == GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIO == GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIO == GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIO == GPIOG){
			GPIOG_PCLK_DI();
		}
	}
}

/****GPIO INIT/DINIT****/
void GPIO_INIT(GPIO_Handle_t *pGPIOHandle){

	GPIO_PCLKControl(pGPIOHandle->pGPIO,ENABLE );
	uint32_t temp1 , temp2;

	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == ALT_Function ){

					temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber/8;
					temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber%8;
					pGPIOHandle->pGPIO->AFR[temp1] &= ~(0b1111  << (4*temp2));
					pGPIOHandle->pGPIO->AFR[temp1] |= (pGPIOHandle->GPIO_Pinconfig.GPIO_PinAltFunMode <<( 4 *temp2));
				}

	//2.Configure the GPIO SPEED
	pGPIOHandle->pGPIO->OSPEEDR&=  ~(0x3 <<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->OSPEEDR |= (pGPIOHandle->GPIO_Pinconfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));

	//3.Configure the GPIO PUPD
	pGPIOHandle->pGPIO->PUPDR&=  ~(0x3 <<(2* pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGPIOHandle->pGPIO->PUPDR |= (pGPIOHandle->GPIO_Pinconfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));

	//4.Configure the GPIO OTYPE
	pGPIOHandle->pGPIO->OTYPER &= ~(1 << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
	pGPIOHandle->pGPIO->OTYPER |= (pGPIOHandle->GPIO_Pinconfig.GPIO_PinOPType << pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);

	//1.Configure the GPIO MODE
		if (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode <= Analog){
			pGPIOHandle->pGPIO->MODER &= ~(0x3 <<(2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
			pGPIOHandle->pGPIO->MODER |= (pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));

		} else{
			GPIOC->MODER &= ~(3U << (13 * 2));
			if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == FT){
				 EXTI->EXTI_RTSR1 &= ~(1 <<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
				 EXTI->EXTI_FTSR1 |= (1 <<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
			}else if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == RT){
				 EXTI->EXTI_FTSR1 &= ~(1 <<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
				 EXTI->EXTI_RTSR1 |= (1 <<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
			}else if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == RT_FT){
				EXTI->EXTI_FTSR1 |= (1 <<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
				EXTI->EXTI_RTSR1 |= (1 <<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
			}

			temp1 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber/4;
			temp2 = pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber%4;
			SYSCFG_PCLK_EN();
			SYSCFG->SYSCFG_EXTICR[temp1] |=  portcode(pGPIOHandle->pGPIO)<<(4* temp2);
			temp1 =0;
			temp2=0;

			EXTI->EXTI_IMR1 |= (1 <<pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
		}


}


void GPIO_DENIT(GPIO_RegDef_t *pGPIO){
			if (pGPIO == GPIOA){
				GPIOA_REG_RST();
			}else if (pGPIO == GPIOB){
				GPIOB_REG_RST();
			}else if(pGPIO == GPIOC){
				GPIOC_REG_RST();
			}else if(pGPIO == GPIOD){
				GPIOD_REG_RST();
			}else if(pGPIO == GPIOE){
				GPIOE_REG_RST();
			}else if(pGPIO == GPIOF){
				GPIOF_REG_RST();
			}else if(pGPIO == GPIOG){
				GPIOG_REG_RST();
			}
}



/****GPIO READ/WRITE TO PORT/PIN****/
uint8_t	 GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber){

	return ((uint8_t) ((pGPIO->IDR >>PinNumber ) & 0x1));
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber,uint8_t value){
	if (value == SET){pGPIO->ODR |= (1 << PinNumber);} else
	{pGPIO->ODR &= ~(1 << PinNumber);}
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIO){
	return (uint16_t)pGPIO->IDR;
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIO,uint16_t value){
	pGPIO->ODR |= value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIO, uint8_t PinNumber){
	pGPIO->ODR ^= (1 <<PinNumber);
}


/****GPIO INTERRUPT HANDLING****/
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t ENorDI){

	if(ENorDI == ENABLE){
		if (IRQNumber <=31){
			*NVIC_ISER0 |= (1<<IRQNumber);
		}else if (IRQNumber >31 && IRQNumber <=63){
			*NVIC_ISER1 |= (1<<IRQNumber % 32);
		}else if(IRQNumber >63 &&IRQNumber <96){
			*NVIC_ISER2 |= (1<<IRQNumber% 64);
		}
	}else{
		if (IRQNumber <=31){
			*NVIC_ICER0 |= (1<<IRQNumber);
		}else if (IRQNumber >31 && IRQNumber <=63){
			*NVIC_ICER1 |= (1<<IRQNumber% 32);
		}else if(IRQNumber >63 &&IRQNumber <96){
			*NVIC_ICER2 |= (1<<IRQNumber% 64);
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

	uint8_t temp1,temp2,temp3;
	temp1 = IRQNumber/4;
	temp2 = IRQNumber%4;
	temp3 = (8 * temp2) + (8 - NO_PR_BITS_IMPLEMNTED);
	*(NVIC_IPR + temp1 ) |= (IRQPriority <<temp3);
}

void GPIO_IRQHandling(uint8_t PinNumber){

	/*Acknowledge and clear pending external interrupt requests*/
	if (EXTI->EXTI_PR1 & (1 << PinNumber)){
		EXTI->EXTI_PR1 |= ( 1 << PinNumber);
	}

}
