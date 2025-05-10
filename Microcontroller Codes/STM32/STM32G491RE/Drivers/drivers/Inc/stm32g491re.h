/*
 * stm32g491re.h
 *
 *  Created on: Feb 9, 2025
 *      Author: ASUS
 */

#ifndef INC_STM32G491RE_H_
#define INC_STM32G491RE_H_

#include <stdint.h>

#define  NO_PR_BITS_IMPLEMNTED					4



/*** Macros for APB prescaler***/
#define HCLK_DIV_2			4
#define HCLK_DIV_4			5
#define HCLK_DIV_8			6
#define HCLK_DIV_16			7


#define SYSCLK_DIV_2		8
#define SYSCLK_DIV_4		9
#define SYSCLK_DIV_8		10
#define SYSCLK_DIV_16		11
#define SYSCLK_DIV_64		12
#define SYSCLK_DIV_128		13
#define SYSCLK_DIV_256		14
#define SYSCLK_DIV_512		15

#define HSE_CLK_FREQ		24000000U
#define HSI_CLK_FRWQ		32000000U
#define PCLK1_FREQ			16000000U
#define SYSCLK_FREQ			16000000U

/********Interrupt Set-Enable Registers*********/
#define  NVIC_ISER0								((volatile uint32_t *)0xE000E100U)
#define  NVIC_ISER1								((volatile uint32_t *)0xE000E104U)
#define  NVIC_ISER2								((volatile uint32_t *)0xE000E108U)

/********Interrupt Clear-Enable Registers**********/
#define  NVIC_ICER0								((volatile uint32_t *)0XE000E180U)
#define  NVIC_ICER1								((volatile uint32_t *)0xE000E184U)
#define  NVIC_ICER2								((volatile uint32_t *)0xE000E188U)

/****** Interrupt Priority Register*********/
#define NVIC_IPR								((volatile uint32_t *)0xE000E400U)


/*********BASE ADDRESSES************/
#define FLASH_BASEADDR							0x08000000U
#define SRAM1_BASEADDR							0x20000000U
#define SRAM2_BASEADDR							0x20014000U
#define CCMRAM_BASEADDR							0x10000000U



#define PERIPHERAL_BASEADDR						0x40000000U
#define APB1_BASEADDR							PERIPHERAL_BASEADDR
#define APB2_BASEADDR							0x40010000U
#define AHB1_BASEADDR 							0x40020000U
#define AHB2_BASEADDR							0x48000000U


/*APB1 peripherals base address*/
#define SPI2_BASEADDR							0x40003800U
#define SPI3_BASEADDR							0x40003C00U

#define USART2_BASEADDR							0x40004400U
#define USART3_BASEADDR							0x40004800U
#define UART4_BASEADDR							0x40004C00U
#define UART5_BASEADDR							0x40005000U


#define I2C1_BASEADDR							0x40005400U
#define I2C2_BASEADDR							0x40005800U
#define I2C3_BASEADDR							0x40007800U
#define I2C4_BASEADDR							0x40008400U


/*APB2 peripherals base address*/
#define SYSCFG_BASEADDR							APB2_BASEADDR
#define SPI1_BASEADDR							0x40013000U
#define SPI4_BASEADDR							0x40013C00U

#define USART1_BASEADDR 						0x40013800U

#define EXTI_BASEADDR							0x40010400U

/*AHB1 peripherals base address*/
#define RCC_BASEADDR							0x40021000U

/*AHB2 peripherals base address*/
#define GPIOA_BASEADDR							AHB2_BASEADDR
#define GPIOB_BASEADDR							0x48000400U
#define GPIOC_BASEADDR							0x48000800U
#define GPIOD_BASEADDR							0x48000C00U
#define GPIOE_BASEADDR							0x48001000U
#define GPIOF_BASEADDR							0x48001400U
#define GPIOG_BASEADDR							0x48001800U


#define TIM1_BASEADDR							0x40010800U
#define TIM8_BASEADDR							0x40013400U
#define TIM15_BASEADDR							0x40014000U

#define TIM6_BASEADDR							0x40001000U
#define TIM7_BASEADDR							0x40001400U

/******General Purpose Macros*******/
#define ENABLE									1
#define DISABLE									0

#define SET										ENABLE
#define RESET									DISABLE

#define HIGH									ENABLE
#define LOW										DISABLE

#define	ON 										SET
#define	OFF										RESET
/*
	Register Definition Structure for System Configuration Registers
*/
typedef struct{

	uint32_t SYSCFG_MEMRMP;
	uint32_t SYSCFG_CFGR1;
	uint32_t SYSCFG_EXTICR[4];
	uint32_t SYSCFG_SCSR;
	uint32_t SYSCFG_CFGR2;
	uint32_t SYSCFG_SWPR;
	uint32_t SYSCFG_SKR;

}SYSCGF_RegDef_t;

#define SYSCFG									((SYSCGF_RegDef_t*)SYSCFG_BASEADDR)


/*
	Register Definition Structure for Extended interrupts and events controller.
*/
typedef struct {

	uint32_t EXTI_IMR1;
	uint32_t EXTI_EMR1;
	uint32_t EXTI_RTSR1;
	uint32_t EXTI_FTSR1;
	uint32_t EXTI_SWIER1;
	uint32_t EXTI_PR1;
	uint32_t REV;
	uint32_t REV1;
	uint32_t EXTI_IMR2;
	uint32_t EXTI_EMR2;
	uint32_t EXTI_RTSR2;
	uint32_t EXTI_FTSR2;
	uint32_t EXTI_SWIER2;
	uint32_t EXTI_PR2;


}EXTI_RegDef_t;

#define EXTI									 ((EXTI_RegDef_t*)EXTI_BASEADDR)


/**************************************** GPIO Register Structure**************************************************/
typedef struct {

	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSSR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];
	volatile uint32_t BRR;

}GPIO_RegDef_t;
/**************************************** GPIO Pointer Definition**************************************************/
#define GPIOA									((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB									((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC									((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD									((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE									((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF									((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG									((GPIO_RegDef_t*)GPIOG_BASEADDR)
/**************************************** GPIO Clock Enable****************************************************/
#define GPIOA_PCLK_EN()							(RCC->AHB2ENR |= (1<<0))
#define GPIOB_PCLK_EN()							(RCC->AHB2ENR |= (1<<1))
#define GPIOC_PCLK_EN()							(RCC->AHB2ENR |= (1<<2))
#define GPIOD_PCLK_EN()							(RCC->AHB2ENR |= (1<<3))
#define GPIOE_PCLK_EN()							(RCC->AHB2ENR |= (1<<4))
#define GPIOF_PCLK_EN()							(RCC->AHB2ENR |= (1<<5))
#define GPIOG_PCLK_EN()							(RCC->AHB2ENR |= (1<<6))
/**************************************** GPIO Clock Disable***************************************************/
#define GPIOA_PCLK_DI()							(RCC->AHB2ENR  &= ~(1<<0))
#define GPIOB_PCLK_DI()							(RCC->AHB2ENR  &= ~(1<<1))
#define GPIOC_PCLK_DI()							(RCC->AHB2ENR  &= ~(1<<2))
#define GPIOD_PCLK_DI()							(RCC->AHB2ENR  &= ~(1<<3))
#define GPIOE_PCLK_DI()							(RCC->AHB2ENR  &= ~(1<<4))
#define GPIOF_PCLK_DI()							(RCC->AHB2ENR  &= ~(1<<5))
#define GPIOG_PCLK_DI()							(RCC->AHB2ENR  &= ~(1<<6))
/**************************************** GPIO PORT RESET******************************************************/
#define GPIOA_REG_RST()							do {(RCC->AHB2RSTR |= (1<<0)) ;(RCC->AHB2RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RST()							do {(RCC->AHB2RSTR |= (1<<1)) ;(RCC->AHB2RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RST()							do {(RCC->AHB2RSTR |= (1<<2)) ;(RCC->AHB2RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RST()							do {(RCC->AHB2RSTR |= (1<<3)) ;(RCC->AHB2RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RST()							do {(RCC->AHB2RSTR |= (1<<4)) ;(RCC->AHB2RSTR &= ~(1<<4));}while(0)
#define GPIOF_REG_RST()							do {(RCC->AHB2RSTR |= (1<<5)) ;(RCC->AHB2RSTR &= ~(1<<5));}while(0)
#define GPIOG_REG_RST()							do {(RCC->AHB2RSTR |= (1<<6)) ;(RCC->AHB2RSTR &= ~(1<<6));}while(0)


/*
	Register Definition Structure for Reset and Clock Control.
*/
typedef struct {

	volatile uint32_t 	CR;
	volatile uint32_t 	ICSCR;
	volatile uint32_t 	CFGR;
	volatile uint32_t  	PLLCFGR;
	         uint32_t	RESERVED1;
	         uint32_t	RESERVED2;
	volatile uint32_t 	CIER;
	volatile uint32_t 	CIFR;
	volatile uint32_t 	CICR;
			 uint32_t	RESERVED3;
	volatile uint32_t  	AHB1RSTR;
	volatile uint32_t 	AHB2RSTR;
	volatile uint32_t 	AHB3RSTR;
	         uint32_t	RESERVED4;
	volatile uint32_t 	APB1RSTR1;
	volatile uint32_t 	APB1RSTR2;
	volatile uint32_t 	APB2RSTR;
	         uint32_t	RESERVED5;
	volatile uint32_t 	AHB1ENR;
	volatile uint32_t 	AHB2ENR;
	volatile uint32_t 	AHB3ENR;
			 uint32_t	RESERVED6;
	volatile uint32_t 	APB1ENR1;
	volatile uint32_t 	APB1ENR2;
	volatile uint32_t 	APB2ENR;
	         uint32_t	RESERVED7;
	volatile uint32_t 	AHB1SMENR;
	volatile uint32_t 	AHB2SMENR;
	volatile uint32_t 	AHB3SMENR;
	         uint32_t	RESERVED8;
	volatile uint32_t 	APB1SMENR1;
	volatile uint32_t 	APB1SMENR2;
	volatile uint32_t 	APB2SMENR;
			 uint32_t	RESERVED9;
	volatile uint32_t 	CCIPR;
	         uint32_t	RESERVED10;
	volatile uint32_t 	BDCR;
	volatile uint32_t 	CSR;
	volatile uint32_t 	CRRCR;
	volatile uint32_t 	CCIPR2;

}RCC_RegDef_t;

#define RCC   									((RCC_RegDef_t*)RCC_BASEADDR)



/**************************************** SPI Register Definition**************************************************/
typedef struct {

	volatile uint32_t	 SPIx_CR1;
	volatile uint32_t	 SPIx_CR2;
	volatile uint32_t	 SPIx_SR;
	volatile uint32_t	 SPIx_DR;
	volatile uint32_t	 SPIx_CRCPR;
	volatile uint32_t	 SPIx_RXCRCR;
	volatile uint32_t	 SPIx_TXCRCR;
	volatile uint32_t	 SPIx_I2SCFGR;
	volatile uint32_t	 SPIx_I2SPR;

}SPI_RegDef_t;
/**************************************** SPI Pointer Definition**************************************************/
#define SPI1									((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2									((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3									((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4									((SPI_RegDef_t*)SPI4_BASEADDR)
/**************************************** SPI Clock Enable**************************************************/
#define SPI1_PCLK_EN()							(RCC->APB2ENR  |= (1<<12))
#define SPI2_PCLK_EN()							(RCC->APB1ENR1 |= (1<<14))
#define SPI3_PCLK_EN()							(RCC->APB1ENR1 |= (1<<15))
#define SPI4_PCLK_EN()							(RCC->APB2ENR  |= (1<<15))
/**************************************** SPI Clock Disable**************************************************/
#define SPI1_PCLK_DI()							(RCC->APB2ENR  &= ~ (1<<12))
#define SPI2_PCLK_DI()							(RCC->APB1ENR1 &= ~ (1<<14))
#define SPI3_PCLK_DI()							(RCC->APB1ENR1 &= ~ (1<<15))
#define SPI4_PCLK_DI()							(RCC->APB2ENR  &= ~ (1<<15))
/**************************************** SPI Peripheral RESET******************************************************/
#define SPI1_REG_RST()							do {(RCC->APB2RSTR  |= (1<<12)) ;(RCC->APB2RSTR  &= ~(1<<12));}while(0)
#define SPI2_REG_RST()							do {(RCC->APB1RSTR1 |= (1<<14)) ;(RCC->APB1RSTR1 &= ~(1<<14));}while(0)
#define SPI3_REG_RST()							do {(RCC->APB1RSTR1 |= (1<<15)) ;(RCC->APB1RSTR1 &= ~(1<<15));}while(0)
#define SPI4_REG_RST()							do {(RCC->APB2RSTR  |= (1<<15)) ;(RCC->APB2RSTR  &= ~(1<<15));}while(0)
/****************************************SPIx_CR1 Bit Fields **************************************************/
#define SPIx_CR1_CPHA 							0
#define SPIx_CR1_CPOL 							1
#define SPIx_CR1_MSTR 							2
#define SPIx_CR1_BUADE_RATE_CTL 				3
#define SPIx_CR1_SPE 							6
#define SPIx_CR1_LSB_FIRST						7
#define SPIx_CR1_SSI 							8
#define SPIx_CR1_SSM 							9
#define SPIx_CR1_RX_ONLY 						10
#define SPIx_CR1_CRCL 							11
#define SPIx_CR1_CRCN_EXTI 						12
#define SPIx_CR1_CRC_EN 						13
#define SPIx_CR1_BIDI_EN 						14
#define SPIx_CR1_BIDI_MODE 						15
/****************************************SPIx_CR2 Bit Fields **************************************************/
#define SPIx_CR2_RXDMAEN						0			// Rx buffer DMA enable
#define SPIx_CR2_TXDMAEN						1			// Tx buffer DMA enable
#define SPIx_CR2_SSOE							2			//SS output enable
#define SPIx_CR2_NSSP							3			// NSS pulse management
#define SPIx_CR2_FRF							4			// Frame format
#define SPIx_CR2_ERRIE							5			// Error interrupt enable
#define SPIx_CR2_RXNEIE							6			//RX buffer not empty interrupt enable
#define SPIx_CR2_TXEIE							7			// Tx buffer empty interrupt enable
#define SPIx_CR2_DS								8			//Data size
#define SPIx_CR2_FRXTH							12			//FIFO reception threshold
#define SPIx_CR2_LDMA_RX						13			//Last DMA transfer for reception
#define SPIx_CR2_LDMA_TX						14			//Last DMA transfer for transmission
/****************************************SPIx_SR  SPI STATUS REGISTER **************************************************/
#define SPIx_SR_RXNE	0		//Receive buffer not empty
#define SPIx_SR_TXE		1		// Transmit buffer empty
#define SPIx_SR_CHSIDE  2		// Channel side
#define SPIx_SR_UDR		3		// Under run flag
#define SPIx_SR_CRCERR	4		//CRC error flag
#define SPIx_SR_MODF	5		// Mode fault
#define SPIx_SR_OVR		6		// Overrun flag
#define SPIx_SR_BSY		7		// Busy flag
#define SPIx_SR_FRE		8		// Frame format error
#define SPIx_SR_FRLVL	9		// FIFO reception level,  00: FIFO empty, 01: 1/4 FIFO,10: 1/2 FIFO,  11: FIFO full
#define SPIx_SR_FTLVL	11		// FIFO transmission level,  00: FIFO empty, 01: 1/4 FIFO,10: 1/2 FIFO,  11: FIFO full



/*****************************UART/USART *****************************/
#define DATA_BITS_8			0
#define DATA_BITS_9			1
#define DATA_BITS_7			2

/*
	Register Definition Structure for UART.
*/
typedef struct {

	volatile uint32_t  USART_CR1;
	volatile uint32_t  USART_CR2;
	volatile uint32_t  USART_CR3;
	volatile uint32_t  USART_BRR;
	volatile uint32_t  USART_GTPR;
	volatile uint32_t  USART_RTOR;
	volatile uint32_t  USART_RQR;
	volatile uint32_t  USART_ISR;
	volatile uint32_t  USART_ICR;
	volatile uint32_t  USART_RDR;
	volatile uint32_t  USART_TDR;
	volatile uint32_t  USART_PRESC;

}UART_RegDef_t;

/****************************USART/UART Peripheral Pointer *****************************/
#define USART1									((UART_RegDef_t*) USART1_BASEADDR)
#define USART2									((UART_RegDef_t*) USART2_BASEADDR)
#define USART3									((UART_RegDef_t*) USART3_BASEADDR)
#define UART4									((UART_RegDef_t*) UART4_BASEADDR)
#define UART5									((UART_RegDef_t*) UART5_BASEADDR)

/*******USART/UART Peripheral Clock Enable*********/
#define USART1_PCLK_EN()						(RCC->APB2ENR  |= (1<<14))
#define USART2_PCLK_EN()						(RCC->APB1ENR1 |= (1<<17))
#define USART3_PCLK_EN()						(RCC->APB1ENR1 |= (1<<18))
#define UART4_PCLK_EN()							(RCC->APB1ENR1 |= (1<<19))
#define UART5_PCLK_EN()							(RCC->APB1ENR1 |= (1<<20))


/*******USART/UART Peripheral Clock Disable*********/
#define USART1_PCLK_DI()						(RCC->APB2ENR  &= ~ (1<<14))
#define USART2_PCLK_DI()						(RCC->APB1ENR1 &= ~ (1<<17))
#define USART3_PCLK_DI()						(RCC->APB1ENR1 &= ~ (1<<18))
#define UART4_PCLK_DI()							(RCC->APB1ENR1 &= ~ (1<<19))
#define UART5_PCLK_DI()							(RCC->APB1ENR1 &= ~ (1<<20))

/*******USART/UART Control Register 1 Bit Fields*********/
#define USART_CR1_UE				0
#define USART_CR1_UESM				1
#define USART_CR1_RE				2
#define USART_CR1_TE				3
#define USART_CR1_IDLEIE			4
#define USART_CR1_RXFNEIE			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXFNFIE			7
#define USART_CR1_PEIE				8
#define USART_CR1_PS				9
#define USART_CR1_PCE				10
#define USART_CR1_WAKE				11
#define USART_CR1_M0				12
#define USART_CR1_MME				13
#define USART_CR1_CMIE				14
#define USART_CR1_OVER8				15
#define USART_CR1_DEDT				16
#define USART_CR1_DEAT				21
#define USART_CR1_RTOIE				26
#define USART_CR1_EOBIE				27
#define USART_CR1_M1				28
#define USART_CR1_FIFOEN			29
#define USART_CR1_TXFEIE			30
#define USART_CR1_RXFFIE			31

/*******USART/UART Control Register 2 Bit Fields*********/
#define USART_CR2_SLVEN				0
#define USART_CR2_DIS_NSS			3
#define USART_CR2_ADDM7				4
#define USART_CR2_LBDL				5
#define USART_CR2_LBDIE				6
#define USART_CR2_LBCL				8
#define USART_CR2_CPHA				9
#define USART_CR2_CPOL				10
#define USART_CR2_CLKEN				11
#define USART_CR2_STOP				12
#define USART_CR2_LINEN				14
#define USART_CR2_SWAP				15
#define USART_CR2_RXINV				16
#define USART_CR2_TXINV				17
#define USART_CR2_DATAINV			18
#define USART_CR2_MSBFIRST			19
#define USART_CR2_ABREN				20
#define USART_CR2_ABRMOD			21
#define USART_CR2_RTOEN				23
#define USART_CR2_ADD				24

/*******USART/UART Control Register 3 Bit Fields*********/
#define USART_CR3_TXFTCFG           29
#define USART_CR3_RXFTIE            28
#define USART_CR3_RXFTCFG           25
#define USART_CR3_TCBGTIE           24
#define USART_CR3_TXFTIE            23
#define USART_CR3_WUFIE             22
#define USART_CR3_WUS               20
#define USART_CR3_SCARCNT           17
#define USART_CR3_DEP               15
#define USART_CR3_DEM               14
#define USART_CR3_DDRE              13
#define USART_CR3_OVRDIS            12
#define USART_CR3_ONEBIT            11
#define USART_CR3_CTSIE             10
#define USART_CR3_CTSE              9
#define USART_CR3_RTSE              8
#define USART_CR3_DMAT              7
#define USART_CR3_DMAR              6
#define USART_CR3_SCEN              5
#define USART_CR3_NACK              4
#define USART_CR3_HDSEL             3
#define USART_CR3_IRLP              2
#define USART_CR3_IREN              1
#define USART_CR3_EIE               0

/*******USART/UART ISR Bit Fields*********/
#define USART_ISR_TXFT				27
#define USART_ISR_RXFT				26
#define USART_ISR_TCBGT				25
#define USART_ISR_RXFF				24
#define USART_ISR_TXFE				23
#define USART_ISR_REACK				22
#define USART_ISR_TEACK				21
#define USART_ISR_WUF				20
#define USART_ISR_RWU				19
#define USART_ISR_SBKF				18
#define USART_ISR_CMF				17
#define USART_ISR_BUSY				16
#define USART_ISR_ABRF				15
#define USART_ISR_ABRE				14
#define USART_ISR_UDR				13
#define USART_ISR_EOBF				12
#define USART_ISR_RTOF				11
#define USART_ISR_CTS				10
#define USART_ISR_CTSIF				9
#define USART_ISR_LBDF				8
#define USART_ISR_TXFNF				7
#define USART_ISR_TC				6
#define USART_ISR_RXFNE				5
#define USART_ISR_IDLE				4
#define USART_ISR_ORE				3
#define USART_ISR_NE				2
#define USART_ISR_FE				1
#define USART_ISR_PE				0


/*APB2 peripheral clk en*/
#define SYSCFG_PCLK_EN()						(RCC->APB2ENR  |= (1<<0))



/**************************************** I2C Register Definition**************************************************/

/*
	Register Definition Structure for I2C.
*/
typedef struct{

	volatile uint32_t I2C_CR1;
	volatile uint32_t I2C_CR2;
	volatile uint32_t I2C_OAR1;
	volatile uint32_t I2C_OAR2;
	volatile uint32_t I2C_TIMINGR;
	volatile uint32_t I2C_TIMEOUTR;
	volatile uint32_t I2C_ISR;
	volatile uint32_t I2C_ICR;
	volatile uint32_t I2C_PECR;
	volatile uint32_t I2C_RXDR;
	volatile uint32_t I2C_TXDR;

}I2C_RegDef_t;

/**************************************** I2C Pointer Definition**************************************************/
#define I2C1									((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2									((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3									((I2C_RegDef_t*)I2C3_BASEADDR)
#define I2C4									((I2C_RegDef_t*)I2C4_BASEADDR)
/**************************************** I2C Peripheral Clock Enable**************************************************/
#define I2C1_PCLK_EN()							(RCC->APB1ENR1 |= (1<<21))
#define I2C2_PCLK_EN()							(RCC->APB1ENR1 |= (1<<22))
#define I2C3_PCLK_EN()							(RCC->APB1ENR1 |= (1<<30))
#define I2C4_PCLK_EN()							(RCC->APB1ENR2 |= (1<<1))
/**************************************** I2C Peripheral Clock Disable**************************************************/
#define I2C1_PCLK_DI()							(RCC->APB1ENR1 &= ~ (1<<21))
#define I2C2_PCLK_DI()							(RCC->APB1ENR1 &= ~ (1<<22))
#define I2C3_PCLK_DI()							(RCC->APB1ENR1 &= ~ (1<<30))
#define I2C4_PCLK_DI()							(RCC->APB1ENR2 &= ~ (1<<1))
/**********************RCC CCIPR I2Cx bit positions(to ENABLE/DISABLE peripheral clock)********************/
#define I2C1SCL		12
#define I2C2SCL		14
#define	I2C3SCL		16
/***********************I2Cx Interrupt Number***************************/
#define IRQNumber_I2C1_EV							31
#define IRQNumber_I2C1_ER							32

#define IRQNumber_I2C2_EV							33
#define IRQNumber_I2C2_ER							34

#define IRQNumber_I2C3_EV							92
#define IRQNumber_I2C3_ER							93

#define IRQNumber_I2C4_EV							82
#define IRQNumber_I2C4_ER							83

/*************EXTI Line Connection*****************/
#define EXTI_LINE_I2C1							23
#define EXTI_LINE_I2C2							24
#define EXTI_LINE_I2C3							27
#define EXTI_LINE_I2C4							42






#define portcode(x)								((x == GPIOA)?0:\
												 (x == GPIOB)?1:\
												 (x == GPIOC)?2:\
												 (x == GPIOD)?3:\
								                 (x == GPIOE)?4:\
								                 (x == GPIOF)?5:\
												 (x == GPIOG)?6:0 )

/*******************Interrupt position**********************/
#define IRQ_NO_EXTI0							6
#define IRQ_NO_EXTI1							7
#define IRQ_NO_EXTI2							8
#define IRQ_NO_EXTI3							9
#define IRQ_NO_EXTI4							10
#define IRQ_NO_EXTI9_5							23
#define IRQ_NO_EXTI15_10						40

#define IRQ_NO_SPI1								35
#define IRQ_NO_SPI2								36
#define IRQ_NO_SPI3								51
#define IRQ_NO_SPI4								84






/**************************************** Simple Timer (TIM6/TIM7)**************************************************/

typedef struct{
	volatile uint32_t	TIMx_CR1;
	volatile uint32_t	TIMx_CR2;
	volatile uint32_t	RESERVED1;
	volatile uint32_t	TIMx_DIER;
	volatile uint32_t	TIMx_SR;
	volatile uint32_t	TIMx_EGR;
	volatile uint32_t	RESERVED2;
	volatile uint32_t	RESERVED3;
	volatile uint32_t	RESERVED4;
	volatile uint32_t	TIMx_CNT;
	volatile uint32_t	TIMx_PSC;
	volatile uint32_t	TIMx_ARR;
}Simple_TIMx_t;

/**************************************** Simple Timer (TIM6/TIM7) Pointer**************************************************/
#define TIM6									((Simple_TIMx_t*)0x40001000U)
#define TIM7									((Simple_TIMx_t*)0x40001400U)

/**************************************** Simple Timer (TIM6/TIM7) Peripheral clock Enable **************************************************/
#define TIM6_PCLK_EN()							(RCC->APB1ENR1 |= (1<<4))
#define TIM7_PCLK_EN()							(RCC->APB1ENR1 |= (1<<5))


#define TIM6_IRQ_NO			54
#endif /* INC_STM32G491RE_H_ */
