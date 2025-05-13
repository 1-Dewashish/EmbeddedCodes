/*
 * stm32g491re_i2c_driver.h
 *
 *  Created on: Mar 7, 2025
 *      Author: ASUS
 */

#ifndef INC_STM32G491RE_I2C_DRIVER_H_
#define INC_STM32G491RE_I2C_DRIVER_H_

/* Direction control for I2C communication */
#define READ        1
#define WRITE       0

/* TIMINGR register bit positions */
#define I2C_TIMINGR_PRESC      28
#define I2C_TIMINGR_SCLDEL     20
#define I2C_TIMINGR_SDADEL     16
#define I2C_TIMINGR_SCLH       8
#define I2C_TIMINGR_SCLL       0

/* CR1 register bit positions */
#define I2C_CR1_PE_BIT             0
#define I2C_CR1_TXIE_BIT           1
#define I2C_CR1_RXIE_BIT           2
#define I2C_CR1_ADDRIE_BIT         3
#define I2C_CR1_NACKIE_BIT         4
#define I2C_CR1_STOPIE_BIT         5
#define I2C_CR1_TCIE_BIT           6
#define I2C_CR1_ERRIE_BIT          7
#define I2C_CR1_SBC_BIT            16
#define I2C_CR1_NOSTRETCH_BIT      17
#define I2C_CR1_WUPEN_BIT          18
#define I2C_CR1_GCEN_BIT           19

/* CR2 register bit positions */
#define I2C_CR2_SLAVEADDR_BIT      0
#define I2C_CR2_RD_WRN_BIT         10
#define I2C_CR2_START_BIT          13
#define I2C_CR2_STOP_BIT           14
#define I2C_CR2_NACK_BIT           15
#define I2C_CR2_NBYTES_BIT         16
#define I2C_CR2_AUTOEND_BIT        25
#define I2C_CR2_PECBYTE_BIT        26
// #define I2C_CR2__BIT             // (Possibly reserved or unused)

/* ISR register bit positions */
#define I2C_ISR_TXE_BIT            0
#define I2C_ISR_TXIS_BIT           1
#define I2C_ISR_RXNE_BIT           2
#define I2C_ISR_ADDR_BIT           3
#define I2C_ISR_NACKF_BIT          4
#define I2C_ISR_STOPF_BIT          5
#define I2C_ISR_TC_BIT             6
#define I2C_ICR_TCR_BIT            7
#define I2C_ICR_BERR_BIT           8
#define I2C_ICR_ARLO_BIT           9
#define I2C_ICR_OVR_BIT            10
#define I2C_ICR_PECERR_BIT         11
#define I2C_ICR_TIMEOUT_BIT        12
#define I2C_ISR_BUSY_BIT           15
#define I2C_ICR_DIR_BIT            16
#define I2C_ICR_ADDCODE_BIT        17

/* ICR register bit positions (Clear Flag bits) */
#define I2C_ICR_ADDRCF_BIT         3
#define I2C_ICR_NACKCF_BIT         4
#define I2C_ICR_STOPCF_BIT         5
#define I2C_ICR_BERRCF_BIT         8
#define I2C_ICR_ARLOCF_BIT         9
#define I2C_ICR_OVRCF_BIT          10
#define I2C_ICR_PECCF_BIT          11
#define I2C_ICR_TIMOUTCF_BIT       12
#define I2C_ICR_ALERTCF_BIT        13

/*
 * NOTE: There are 4 I2C instances, but only the first three (I2C1, I2C2, I2C3)
 * can have their clock source selected. All I2C instances derive their clock from PCLK1.
 */

/* Macros to select the clock source for I2Cx from RCC_CCIPR */
#define I2Cx_CLK_PCLK              0
#define I2Cx_CLK_SYSCLK            1
#define I2Cx_CLK_HSI16             2

/* I2C configuration structure */
typedef struct {
    uint32_t I2CCKL;
    uint32_t I2C_SETUP_TIME;
    uint32_t I2C_HOLD_TIME;
    uint32_t I2C_SCL_HIGH_PERIOD;
    uint32_t I2C_SCL_LOW_PERIOD;
    uint32_t I2C_SPEED;
} I2C_Config_t;

/* I2C handle structure */
typedef struct {
    I2C_Config_t    I2CConfig;
    I2C_RegDef_t    *pI2C;
    uint32_t        TX_Length;
} I2C_Handle_t;

/* Function prototypes */

/* Initialize the I2C peripheral */
void I2C_INIT(I2C_Handle_t *I2CHandle);

/* Start condition generation and slave addressing */
void StartBitGeneration(I2C_RegDef_t *I2Cx, uint32_t Len, uint8_t SlaveAddr, uint8_t Read_or_Write);

/* Blocking send function */
void SendData(I2C_RegDef_t *I2Cx, uint8_t *pTXBuffer, uint32_t Len);

/* Interrupt-based send */
void SendDataIT(I2C_Handle_t *I2Cx, uint8_t *pTXBuffer);

/* Master transmit in blocking mode */
void I2C_MasterSendData(I2C_RegDef_t *I2Cx, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr);

/* Master receive in blocking mode */
void I2C_MasterReciveData(I2C_RegDef_t *I2Cx, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RegAddr);

/* Master transmit in interrupt mode */
void I2C_MasterSendData_IT(I2C_Handle_t *I2Cx, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RegAddr);

/* Master receive in interrupt mode */
void I2C_MasterReciveData_IT(I2C_RegDef_t *I2Cx, uint8_t *pRXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t RegAddr);

/* Configure I2C interrupt */
void I2Cx_IRQConfig(uint8_t IRQNumber, uint8_t ENorDI);

/* Configure I2C interrupt priority */
void I2Cx_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/* Scan for available I2C devices */
void I2CScanDevices(I2C_RegDef_t *I2Cx);

#endif /* INC_STM32G491RE_I2C_DRIVER_H_ */
