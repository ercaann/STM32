/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: 24 Nis 2020
 *      Author: ERCAN
 */

#include <spi_driver.h>

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
{
	if(pSPIx == SPI1)
	{
		SPI1_PCLK_EN();

	}else if(pSPIx == SPI2)
	{
		SPI2_PCLK_EN();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_PCLK_EN();
	}
}
	else if (EnorDi == DISABLE)
	{

	if(pSPIx == SPI1)
	{
		SPI1_PCLK_DI();

	}else if(pSPIx == SPI2)
	{
		SPI2_PCLK_DI();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_PCLK_DI();
	}
	}
}

void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	SPI_PClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure the device mode
	pSPIHandle->pSPIx->CR1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	// Configure the Bus
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){

		(pSPIHandle->pSPIx->CR1) &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){

		(pSPIHandle->pSPIx->CR1) |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SRX){

		(pSPIHandle->pSPIx->CR1) &= ~(1 << SPI_CR1_BIDIMODE);
		(pSPIHandle->pSPIx->CR1) |= (1 << SPI_CR1_RXONLY);
	}

	// Configure the SPI Serial Clock Speed baud rate
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SClkSpeed << SPI_CR1_BR);

	// Configure the CPOL
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// Configure the CPHA
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// Configure the DFF
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// Configure the SSM Slave Management Software
	pSPIHandle->pSPIx->CR1 |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		{
			SPI1_REG_RESET();

		}else if(pSPIx == SPI2)
		{
			SPI2_REG_RESET();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_REG_RESET();
		}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName) 	// check Flag bit in SR
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

uint8_t SPI_CheckSS(SPI_RegDef_t *pSPIx){

	if(pSPIx->CR1 & (1 << SPI_CR1_SSI)){  // check SS pin

		return SET;
	}
	return RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len )
{
	while(Len > 0)
{

	while(SPI_GetFlagStatus(pSPIx,SPI_FLAG_TXE) == FLAG_RESET ); 	// wait until TXE is set

	if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ) 		// check the DFF bit in CR1

	{
		//16 bit DFF load the data into the DR
		pSPIx->DR =  *((uint16_t*)pTxBuffer);
		Len--;
		Len--;
		(uint16_t*)pTxBuffer++;

	}else
	{
		//8 bit DFF
		pSPIx->DR = *pTxBuffer;
		Len--;
		pTxBuffer++;
	}
}

}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len )
{
	while(Len > 0)
{

	while(SPI_GetFlagStatus(pSPIx, SPI_SR_RXNE) == FLAG_RESET ); 	// wait until TXE is set

	if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ) 		// check the DFF bit in CR1

	{
		//16 bit DFF load the data in to the DR
		pSPIx->DR =  *((uint16_t*)pRxBuffer);
		Len--;
		Len--;
		(uint16_t*)pRxBuffer++;

	}else
	{
		//8 bit DFF
		pSPIx->DR = *pRxBuffer;
		Len--;
		pRxBuffer++;
	}
}

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SPE);
	}
}

/*
 * Master mode fault occurs when the master device has its NSS pin pulled low (in NSS hardware mode)
 * or SSI bit low (in NSS software mode), this automatically sets the MODF bit
 * this makes NSS signal internally high and avoids MODF error
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |=  (1 << SPI_CR1_SSI);

	}else
	{
		pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
	}
}

/*
 * Hardware NSS management (SSM = 0)
 * NSS output enabled (SSM = 0, SSOE = 1)  when the device operates in master mode and NSS signal is driven low
 * NSS output disabled (SSM = 0, SSOE = 0) This configuration allows multimaster capability
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR1 |=  (1 << SPI_CR2_SSOE);

	}else
	{
			pSPIx->CR1 &=  ~(1 << SPI_CR2_SSOE);
	}
}


void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
	if(IRQNumber <= 31){

		// program ISER0 register
		*NVIC_ISER0 |= ( 1 << IRQNumber );
	}

	else if(IRQNumber > 31 && IRQNumber < 64){ // 32 to 63

		//program ISER1 register
		*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );
	}
	else if(IRQNumber > 63 && IRQNumber < 96){ // 64 to 95

		//program ISER2 register
		*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
	}

	}else{

	if(IRQNumber <= 31){

		// program ICER0 register
		*NVIC_ICER0 |= ( 1 << IRQNumber );

	}

	else if(IRQNumber > 31 && IRQNumber < 64){ // 32 to 63

		//program ICER1 register
		*NVIC_ICER1 |= ( 1 << IRQNumber % 32 );

	}
	else if(IRQNumber > 63 && IRQNumber < 96){ // 64 to 95

		//program ICER2 register
		*NVIC_ICER2 |= ( 1 << IRQNumber % 64 );

	}

	}

}
void SPI_IRQPriorityConfig(uint32_t IRQPriority, uint8_t IRQNumber)
{
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;
	uint8_t shift_amount = (8 * IPRx_Section) + (8 - NO_IPR_BITS_IMPLEMENTED); //unimplemented low-order 4 bits IPR register
	*(NVIC_IPR + IPRx) |=  (IRQPriority << shift_amount);

}

uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pHandle->TxState;

	if(state != SPI_BUSY_IN_TX){

		// save Tx buffer address and Len information
		pHandle->pTxBuffer = pTxBuffer;
		pHandle->TxLen = Len;

		// SPI state busy in transmission
		pHandle->TxState = SPI_BUSY_IN_TX;

		//whenever TXE flag set enable TXEIE bit
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pHandle->RxState;

	if(state != SPI_BUSY_IN_RX){

		// save Rx buffer address and Len information
		pHandle->pRxBuffer = pRxBuffer;
		pHandle->RxLen = Len;

		// SPI state busy in reception
		pHandle->RxState = SPI_BUSY_IN_RX;

		//whenever RXNE flag set enable TXEIE bit
		pHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}

	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;

	// check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2)
	{

		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	//check for OVR flag
	//An overrun condition occurs when the master device has sent data bytes and
	//the slave device has not cleared the RXNE bit resulting from the previous data byte transmitted
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
		//check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}


static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
	{
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen --;
		pSPIHandle->RxLen --;
		(uint16_t*)pSPIHandle->pRxBuffer++;

	}else
	{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}


static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	
	//Clearing the OVR bit is done by a read from the SPI_DR register
	//followed by a read access to the SPI_SR register
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}


void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

}

