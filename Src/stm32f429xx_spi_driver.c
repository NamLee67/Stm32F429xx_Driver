/*
 * stm32f429xx_spi_driver.c
 *
 *  Created on: Aug 29, 2024
 *      Author: ACER
 */
#include "stm32f429xx_spi_driver.h"

static void spi_txe_it_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_it_handle(SPI_Handle_t *pSPIHandle);
static void spi_ove_err_it_handle(SPI_Handle_t *pSPIHandle);
/******************************************************************
 * @fn						- SPI_PeriClockControl
 *
 * @brief					- This function is toggle data from pin of port GPIOx
 *
 * @param[in]				- Base address of the GPIOx peripheral
 * @param[in]				- Pin Number of GPIOx want to toggle
 * @param[in]				-
 *
 * @return					- none
 *
 * @note					- none
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}

		}else
		{
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}else if(pSPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}else if(pSPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}else if(pSPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}

/******************************************************************
 * @fn						- SPI_init
 *
 * @brief					- This function is toggle data from pin of port GPIOx
 *
 * @param[in]				- Base address of the GPIOx peripheral
 * @param[in]				- Pin Number of GPIOx want to toggle
 * @param[in]				-
 *
 * @return					- none
 *
 * @note					- none
 */
void SPI_init(SPI_Handle_t *pSPIHandle)
{
	// first lets configure the SPI_CR1 register
	 uint32_t tempreg = 0;

	 // enable clock
	 SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	 // 1. configure the device mode
	 tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	 // 2. configure the bus
	 if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	 {
		 // clear the BIDIMODE ( bit15 )
		 tempreg &= ~(1 << 15);
	 }
	 if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	 {
		 // enable the BIDIMODE ( bit15 )
		 tempreg |= (1 << 15);
	 }
	 if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX)
	 {
		 // clear the BIDIMODE ( bit15 )
		 tempreg &= ~(1 << 15);
		 // set RXONLY ( bit 10 )
		 tempreg |= (1 << 10);
	 }

	 // 3. configure the serial clock speed ( baud rate )
	 tempreg |= pSPIHandle->SPIConfig.SPI_CLKSpeed << 3;

	 // 4. configure the DFF
	 tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	 // 5. configure the CPOL
	 tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << 1;

	 // 6. configure the CPHA
	 tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << 0;

	 pSPIHandle->pSPIx->CR1 = tempreg;
}

/******************************************************************
 * @fn						- SPI_DeInit
 *
 * @brief					- This function is toggle data from pin of port GPIOx
 *
 * @param[in]				- Base address of the GPIOx peripheral
 * @param[in]				- Pin Number of GPIOx want to toggle
 * @param[in]				-
 *
 * @return					- none
 *
 * @note					- none
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

uint8_t SPI_FlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/******************************************************************
 * @fn						- SPI_SendData
 *
 * @brief					- This function is toggle data from pin of port GPIOx
 *
 * @param[in]				- Base address of the GPIOx peripheral
 * @param[in]				- Pin Number of GPIOx want to toggle
 * @param[in]				-
 *
 * @return					- none
 *
 * @note					- none
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint8_t Len)
{
	// wait to len = 0
	while(Len > 0)
	{
		// 1. wait until TXE is set ( means TX Buffer is empty )
		while(SPI_FlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bits DFF
			// load 2 byte data from TxBuffer into DR register
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			// increment the buffer
			(uint16_t*)pTxBuffer++;
			Len--;
			Len--;
		}else
		{
			// 8 bits DFF
			// load 1 byte data into DR register
			pSPIx->DR = *(pTxBuffer);
			// increment the buffer
			pTxBuffer++;
			Len--;
		}
	}
}


/******************************************************************
 * @fn						- SPI_ReceiveData
 *
 * @brief					- This function is toggle data from pin of port GPIOx
 *
 * @param[in]				- Base address of the GPIOx peripheral
 * @param[in]				- Pin Number of GPIOx want to toggle
 * @param[in]				-
 *
 * @return					- none
 *
 * @note					- none
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint8_t Len)
{
	// wait to len = 0
	while(Len > 0)
	{
		// 1. wait until RXNE is set ( means RX Buffer is non empty )
		while(SPI_FlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bits DFF
			// load 2 byte data from DR register into RxBuffer
			*((uint16_t*)pRxBuffer) =pSPIx->DR;
			// increment the buffer
			(uint16_t*)pRxBuffer++;
			Len--;
			Len--;
		}else
		{
			// 8 bits DFF
			// load 1 byte data from DR register into RxBuffer
			*pRxBuffer = pSPIx->DR;
			// increment the buffer
			pRxBuffer++;
			Len--;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint8_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IT_TX)
	{
		//1. Save the Tx buffer address and Len information in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		//   no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IT_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE lag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
	//4. Data transmission will be handled by the ISR code
}
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint8_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IT_RX)
	{
		//1. Save the Tx buffer address and Len information in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2. Mark the SPI state as busy in transmission so that
		//   no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IT_RX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE lag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
	//4. Data transmission will be handled by the ISR code
}
/******************************************************************
 * @fn						- GPIO_ToggleOutputPin
 *
 * @brief					- This function is toggle data from pin of port GPIOx
 *
 * @param[in]				- Base address of the GPIOx peripheral
 * @param[in]				- Pin Number of GPIOx want to toggle
 * @param[in]				-
 *
 * @return					- none
 *
 * @note					- none
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{

		if(IRQNumber <= 31)
		{
			// program in ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			// program in ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// program in ISER2 register
			*NVIC_ISER1 |= (1 << IRQNumber % 64);
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	uint8_t temp1 = IRQNumber / 4;
	uint8_t temp2 = IRQNumber % 4;

	uint8_t shift_amount = (8 * temp2) + (8 -  NO_PR_BITS_IMPLEMNET);
	*(NVIC_PR_BASEADDR + temp1)|= (IRQPriority << shift_amount);
}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

	uint8_t temp1, temp2;

	// check for txe
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		// handle TXE
		spi_txe_it_handle(pHandle);
	}

	// check for rxne
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		// handle TXE
		spi_rxne_it_handle(pHandle);
	}

	// check for error
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		// handle TXE
		spi_ove_err_it_handle(pHandle);
	}
}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

void SPI_SSIControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

void SPI_SSOEControl(SPI_RegDef_t *pSPIx, uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE);
	}
}

static void spi_txe_it_handle(SPI_Handle_t *pSPIHandle)
{
	// 1. check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bits DFF
		// load 2 byte data from TxBuffer into DR register
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		// increment the buffer
		(uint16_t*)pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
	}else
	{
		// 8 bits DFF
		// load 1 byte data into DR register
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		// increment the buffer
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}

	if(!pSPIHandle->TxLen)
	{
		pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE );
		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLen = 0;
		pSPIHandle->TxState = SPI_READY;
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}
static void spi_rxne_it_handle(SPI_Handle_t *pSPIHandle)
{
	// 2. check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bits DFF
		// load 2 byte data from DR register into RxBuffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		// increment the buffer
		(uint16_t*)pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
	}else
	{
		// 8 bits DFF
		// load 1 byte data from DR register into RxBuffer
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		// increment the buffer
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}

	if(!pSPIHandle->RxLen)
	{
		pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE );
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;
		SPI_AppEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ove_err_it_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IT_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	//2. inform the application
	SPI_AppEventCallback(pSPIHandle, SPI_EVENT_OVR_CMPLT);
	(void)temp;
}

__weak void SPI_AppEventCallback(SPI_Handle_t *pSPIHandle, uint8_t event){

}

