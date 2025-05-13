/*
 * stm32g491re_i2c_driver.c
 *
 *  Created on: Mar 7, 2025
 *      Author: ASUS
 */


#include "stm32g491re.h"
#include "stm32g491re_i2c_driver.h"
#include <STDIO.h>

uint32_t temp_len=0;

/********Based on the clock source decided by user , following function will be executed*********/

void HSI16_I2C_CLK(I2C_Handle_t *I2CHandle){
	//1. Calculating I2C Timing Prescaler
	uint8_t PRESC = (HSI_CLK_FRWQ/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= PRESC << I2C_TIMINGR_PRESC;

	//2. Calculating Data setup time
	uint8_t SCLDEL = (I2CHandle->I2CConfig.I2C_SETUP_TIME/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SCLDEL << I2C_TIMINGR_SCLDEL;

	//3. Calculating Data hold time
	uint8_t SDADEL = (I2CHandle->I2CConfig.I2C_HOLD_TIME/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SDADEL << I2C_TIMINGR_SDADEL;

	//5. Calculating SCL high period (master mode)
	uint8_t SCLH = (I2CHandle->I2CConfig.I2C_SCL_HIGH_PERIOD/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SCLH << I2C_TIMINGR_SCLH;

	//5. Calculating  SCL low period (master mode)
	uint8_t SCLL = (I2CHandle->I2CConfig.I2C_SCL_LOW_PERIOD/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SCLL << I2C_TIMINGR_SCLL;

}
void PCKL_I2C_CLK(I2C_Handle_t *I2CHandle){

	// HSI/HSE/PLL-> SYSCLK(PRESCALER) -> AHB (PRESCALER) -> APB1(PRESCLAR) -> PCLK

	//1. Calculating I2C Timing Prescaler
	uint8_t PRESC = (PCLK1_FREQ/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= PRESC << I2C_TIMINGR_PRESC;

	//2. Calculating Data setup time
	uint8_t SCLDEL = (I2CHandle->I2CConfig.I2C_SETUP_TIME/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SCLDEL << I2C_TIMINGR_SCLDEL;

	//3. Calculating Data hold time
	uint8_t SDADEL = (I2CHandle->I2CConfig.I2C_HOLD_TIME/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SDADEL << I2C_TIMINGR_SDADEL;

	//5. Calculating SCL high period (master mode)
	uint8_t SCLH = (I2CHandle->I2CConfig.I2C_SCL_HIGH_PERIOD/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SCLH << I2C_TIMINGR_SCLH;

	//5. Calculating  SCL low period (master mode)
	uint8_t SCLL = (I2CHandle->I2CConfig.I2C_SCL_LOW_PERIOD/I2CHandle->I2CConfig.I2C_SPEED) -1;
	I2CHandle->pI2C->I2C_TIMINGR |= SCLL << I2C_TIMINGR_SCLL;

}


void SYSCLK_I2C_CLK(I2C_Handle_t *I2CHandle) {
    uint32_t timingr = 0; // Temporary variable

    // Compute PRESC safely
    uint8_t PRESC = (SYSCLK_FREQ / I2CHandle->I2CConfig.I2C_SPEED) - 1;
    if (PRESC > 0xF) PRESC = 0xF;
    timingr |= (PRESC & 0x0F) << 28;

    // Compute SCLDEL using safer unit conversions
    uint8_t SCLDEL = ((I2CHandle->I2CConfig.I2C_SETUP_TIME * PCLK1_FREQ) / 1000000000) - 1;
    if (SCLDEL > 0xF) SCLDEL = 0xF;
    timingr |= (SCLDEL & 0x0F) << 20;

    // Compute SDADEL safely
    uint8_t SDADEL = ((I2CHandle->I2CConfig.I2C_HOLD_TIME * PCLK1_FREQ) / 1000000000) - 1;
    if (SDADEL > 0xF) SDADEL = 0xF;
    timingr |= (SDADEL & 0x0F) << 16;

    // Compute SCL High Period safely
    uint8_t SCLH = ((I2CHandle->I2CConfig.I2C_SCL_HIGH_PERIOD * PCLK1_FREQ) / 1000000000) - 1;
    if (SCLH > 0xFF) SCLH = 0xFF;
    timingr |= (SCLH & 0xFF) << 8;

    // Compute SCL Low Period safely
    uint8_t SCLL = ((I2CHandle->I2CConfig.I2C_SCL_LOW_PERIOD * PCLK1_FREQ) / 1000000000) - 1;
    if (SCLL > 0xFF) SCLL = 0xFF;
    timingr |= (SCLL & 0xFF) << 0;

    // Assign TIMINGR register once
    I2CHandle->pI2C->I2C_TIMINGR = timingr;
}


void I2C_PCLKControl(I2C_RegDef_t *pI2C, uint8_t ENorDI ){
	if (ENorDI == ENABLE){

		if (pI2C == I2C1){
			I2C1_PCLK_EN();
		}else if (pI2C == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2C == I2C3){
			I2C3_PCLK_EN();
		}else if(pI2C == I2C4){
			I2C4_PCLK_EN();
		}

	} else if(ENorDI == DISABLE){

		if (pI2C == I2C1){
			I2C1_PCLK_DI();
		}else if (pI2C == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2C == I2C3){
			I2C3_PCLK_DI();
		}else if(pI2C == I2C4){
			I2C4_PCLK_DI();
		}
	}
}

void I2C_INIT(I2C_Handle_t *I2CHandle){

	/**Setting the clock source for i2c1,i2c2,i2c3**/
	/*Now we need to find the values to written in I2C_TIMINGR.
	 * Calculating these values are different for all three clock source*/

	//////Configure the ACK BIR

	//Peripheral disabled
	I2CHandle->pI2C->I2C_CR1 &= ~( 1 << I2C_CR1_PE_BIT);

	// first set the driserd clk sourfe for repective i2c peripheral and then based on selected clk set the i2c_timmingr
	typedef void (*I2CCLKFuncPTR)(I2C_Handle_t *I2CHandle);

	I2CCLKFuncPTR Func_I2C_CLK[] = {PCKL_I2C_CLK,SYSCLK_I2C_CLK,HSI16_I2C_CLK};


	I2CHandle->pI2C->I2C_TIMINGR = 0;  // Clear TIMINGR before setting values

	if(I2CHandle->pI2C == I2C1){

		RCC->CCIPR = I2CHandle->I2CConfig.I2CCKL << I2C1SCL;
		I2C_PCLKControl(I2CHandle->pI2C,ENABLE);
		Func_I2C_CLK[I2CHandle->I2CConfig.I2CCKL](I2CHandle);

	}
	else if(I2CHandle->pI2C == I2C2){
		RCC->CCIPR = I2CHandle->I2CConfig.I2CCKL << I2C2SCL;
		I2C_PCLKControl(I2CHandle->pI2C,ENABLE);
		I2CHandle->pI2C->I2C_TIMINGR = 0x00C0216C;
		//Func_I2C_CLK[I2CHandle->I2CConfig.I2CCKL](I2CHandle);
	}
	else if(I2CHandle->pI2C == I2C3){
		RCC->CCIPR = I2CHandle->I2CConfig.I2CCKL << I2C3SCL;
		I2C_PCLKControl(I2CHandle->pI2C,ENABLE);
		I2CHandle->pI2C->I2C_TIMINGR = 0x00C0216C;
//		I2C_PCLKControl(I2CHandle->pI2C,ENABLE);
		//Func_I2C_CLK[I2CHandle->I2CConfig.I2CCKL](I2CHandle);
	}

	I2CHandle->pI2C->I2C_CR1 &= ~( 1 << I2C_CR1_PE_BIT);
	I2CHandle->pI2C->I2C_CR1 &= ~( 1 << I2C_CR1_PE_BIT);
	// Peripheral enabled
	I2CHandle->pI2C->I2C_CR1 |= ( 1 << I2C_CR1_PE_BIT);

}

void StartBitGeneration(I2C_RegDef_t *I2Cx,uint32_t Len, uint8_t SlaveAddr,uint8_t Read_or_Write){

	if(Read_or_Write == WRITE){

		// 1. Ensure I2C is not busy
		while (I2Cx->I2C_ISR & (1 << I2C_ISR_BUSY_BIT));

		// 2. Configure slave address and data length
		I2Cx->I2C_CR2 = ((SlaveAddr << 1) ) | ((Len)  << 16);

		// 3. Enable START Bit
		I2Cx->I2C_CR2 |= (1 << I2C_CR2_START_BIT);

	}else{

		/****NO Need to check busy state as it will be repeated start while (I2Cx->I2C_ISR & (1 << I2C_ISR_BUSY_BIT));****/

		// 1. Configure slave address and data length
		I2Cx->I2C_CR2 = ((SlaveAddr << 1) ) | ((Len)  << I2C_CR2_NBYTES_BIT);

		// 2. Set a read transfer
		I2Cx->I2C_CR2 |= (1 << I2C_CR2_RD_WRN_BIT);

		// 3. Enable START Bit
		I2Cx->I2C_CR2 |= (1 << I2C_CR2_START_BIT);
	}
}


void StopBitGeneration(I2C_RegDef_t *I2Cx){

    // 1. Wait for Transmission Complete.
   while (!(I2Cx->I2C_ISR & (1 << I2C_ISR_TC_BIT)));

   // 2. Generate STOP condition.
   I2Cx->I2C_CR2 |= (1 << I2C_CR2_STOP_BIT);

}

void SendData(I2C_RegDef_t *I2Cx, uint8_t *pTXBuffer, uint32_t Len){


	for (uint32_t i = 0; i < Len; i++) {
        while (!(I2Cx->I2C_ISR & (1 << I2C_ISR_TXIS_BIT)));  						// Wait until TXIS is set
        I2Cx->I2C_TXDR = *pTXBuffer; 												// Load data into TXDR
        pTXBuffer++;                 												// Move to the next byte
    }

}
void SendDataIT(I2C_Handle_t *I2C, uint8_t *pTXBuffer){


	//I2C->pI2C->I2C_CR2 |= (1 << I2C_CR2_AUTOEND_BIT );
    I2C->pI2C->I2C_TXDR = pTXBuffer[temp_len - I2C->TX_Length]; 												// Load data into TXDR
    I2C->TX_Length--;


}
void ReciveData(I2C_RegDef_t *I2Cx, uint8_t *pRXBuffer, uint32_t Len){

	for (uint32_t i = 0; i < Len; i++) {
		while (!(I2Cx->I2C_ISR & (1 << I2C_ISR_RXNE_BIT)));							// Wait until RXNE is set
		*pRXBuffer = I2Cx->I2C_RXDR;												// Read data from RXDR
		pRXBuffer++;																// Move to the next byte

	}
}



void I2C_MasterSendData(I2C_RegDef_t *I2Cx, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr) {

	// 1. Start Bit Generation with WRITE transfer.
	StartBitGeneration(I2Cx,Len,SlaveAddr,WRITE);

	// 2. Start Sending Data.
	SendData(I2Cx,pTXBuffer,Len);

    // 3. STOP Bit Generation.
    StopBitGeneration(I2Cx);

}

void I2C_MasterReciveData(I2C_RegDef_t *I2Cx, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t RegAddr){

	// 1. Start Bit Generation with WRITE transfer.
	StartBitGeneration(I2Cx,1,SlaveAddr,WRITE);

	// 2. Sending the register address.
	SendData(I2Cx,&RegAddr,1);

	// 3.Generate repeated start with READ transfer.
	StartBitGeneration(I2Cx,Len,SlaveAddr,READ);

	// 4. Receive the the data in pRXBuffer.
	ReciveData(I2Cx,pRXBuffer,Len);

	// 5. STOP Bit Generation.
    StopBitGeneration(I2Cx);

}


void I2C_MasterSendData_IT(I2C_Handle_t *I2C,uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t RegAddr){


	I2C->pI2C->I2C_CR1 |= (1 << I2C_CR1_TXIE_BIT);
	printf("%s","I2C_MasterSendData_IT\n");

	I2C->TX_Length = Len;
	temp_len = Len;
	// 1. Start Bit Generation with WRITE transfer.
	StartBitGeneration(I2C->pI2C,Len,SlaveAddr,WRITE);
	printf("%s","if this line is printed disabe the txie interrupt");

}

void I2C_MasterReciveData_IT(I2C_RegDef_t *I2Cx,uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RegAddr){

}

void I2Cx_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI){

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
void I2Cx_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){

	uint8_t temp1,temp2,temp3;
	temp1 = IRQNumber/4;
	temp2 = IRQNumber%4;
	temp3 = (8 * temp2) + (8 - NO_PR_BITS_IMPLEMNTED);
	*(NVIC_IPR + temp1 ) |= (IRQPriority <<temp3);

}

/* Scan for available I2C devices */
void I2CScanDevices(I2C_RegDef_t *I2Cx) {

    for (uint8_t addr = 0x00; addr <= 0x7F; addr++) {

        // 1. Clear previous STOPF or NACKF flags, if any
        I2Cx->I2C_ICR |= (1 << I2C_ICR_STOPCF_BIT) | (1 << I2C_ICR_NACKCF_BIT);

        // 2. Configure I2C_CR2:
        //    - Set slave address (left-shifted by 1)
        //    - Set number of bytes to send: 1 (dummy write)
        //    - Set start condition
        //    - Set auto-end mode: generates STOP after NBYTES
        //    - Set direction to write (RD_WRN = 0)
        //    - Enable START
        I2Cx->I2C_CR2 = (addr << 1)             // SADD: 7-bit address
                      | (1 << 16)               // NBYTES = 1
                      | (1 << 13)               // START
                      | (1 << 25);              // AUTOEND

        // 3. Wait until either NACKF or TXIS (transmit interrupt status) flag is set
        while (!(I2Cx->I2C_ISR & ((1 << I2C_ISR_TXIS_BIT) | (1 << I2C_ISR_NACKF_BIT))));

        // 4. If TXIS is set, device ACKed, send a dummy byte
        if (I2Cx->I2C_ISR & (1 << I2C_ISR_TXIS_BIT)) {
            I2Cx->I2C_TXDR = 0x00;  // Send dummy byte

            // Wait until STOPF is set (transfer complete)
            while (!(I2Cx->I2C_ISR & (1 << I2C_ISR_STOPF_BIT)));

            // Clear STOP flag
            I2Cx->I2C_ICR |= (1 << I2C_ICR_STOPCF_BIT);

            printf("Address found is 0x%02X\n", addr);
        }
        else {
            // If NACKF was set, clear it
            I2Cx->I2C_ICR |= (1 << I2C_ICR_NACKCF_BIT);
        }
    }
}

