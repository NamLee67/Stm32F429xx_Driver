/*
 * stm32f429xx_spi_driver.h
 *
 *  Created on: Aug 29, 2024
 *      Author: ACER
 */

#ifndef INC_STM32F429XX_SPI_DRIVER_H_
#define INC_STM32F429XX_SPI_DRIVER_H_

#include "stm32f429xx.h"
/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_CLKSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handling structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;			// this hold the base address of SPIx(1, 2, 3, 4) peripharals
	SPI_Config_t	SPIConfig;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
}SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MASTER 	1
#define SPI_DEVICE_SLAVE	0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			1
#define SPI_BUS_CONFIG_HD			2
#define SPI_BUS_CONFIG_SIMPLEX_RX	3

/*
 * @SPI_CLKSpeed
 */
#define SPI_CLK_SPEED_DEV2			0
#define SPI_CLK_SPEED_DEV4			1
#define SPI_CLK_SPEED_DEV8			2
#define SPI_CLK_SPEED_DEV16			3
#define SPI_CLK_SPEED_DEV32			4
#define SPI_CLK_SPEED_DEV64			5
#define SPI_CLK_SPEED_DEV128		6
#define SPI_CLK_SPEED_DEV256		7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN	1
#define SPI_SSM_DI	0

/*
 * SPI related status flags definition
 */
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)

/*
 *
 */
#define SPI_READY		0
#define SPI_BUSY_IT_RX	1
#define SPI_BUSY_IT_TX	2

/*
 *
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_CMPLT	3

/**********************************************************************************************
 * 								APIs Supported by this driver
 * 				For more information about the APIs check the function definitions
 **********************************************************************************************
 */

/*
 * Peripheral Clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * Init and De-init
 */
void SPI_init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Data sending and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Len);
/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI);
void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI);
void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI);
uint8_t SPI_FlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event);

#endif /* INC_STM32F429XX_SPI_DRIVER_H_ */











