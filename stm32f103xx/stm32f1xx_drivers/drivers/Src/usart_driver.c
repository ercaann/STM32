/*
 * stm32f103xx_usart_driver.c
 *
 *  Created on: 25 May 2020
 *      Author: ERCAN
 */

#include "usart_driver.h"

static void USART_TXEHandleIT(USART_Handle_t *pUSARTHandle);
static void USART_RXNEHandleIT(USART_Handle_t *pUSARTHandle);


void USART_PClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
{
	if(pUSARTx == USART1)
	{
		USART1_PCLK_EN();

	}else if(pUSARTx == USART2)
	{
		USART2_PCLK_EN();
	}
	else if(pUSARTx == USART3)
	{
		USART3_PCLK_EN();
	}
	else if(pUSARTx == UART4)
	{
		UART4_PCLK_EN();
	}
	else if(pUSARTx == UART5)
	{
			UART5_PCLK_EN();
	}
}
	else if (EnorDi == DISABLE)
	{

	if(pUSARTx == USART1)
	{
		USART1_PCLK_DI();

	}else if(pUSARTx == USART2)
	{
		USART2_PCLK_DI();
	}
	else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
	else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
	else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
	}
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	//define APB clock mantissa and fraction values
	uint32_t PCLKx, Mpart, Fpart;
	uint32_t USARTDIV, fraction;

	if(pUSARTx == USART2 || pUSARTx == USART3){
		//USART2 and USART3 APB1 bus
		PCLKx = RCC_PCLK1_GetValue();
	}
	else{
		//USART1 APB2 bus
		PCLKx = RCC_PCLK2_GetValue();
	}

	// over sampling by 16 (default)
	USARTDIV = (PCLKx) / (16 * BaudRate);  // BAUD = FCLK / (16*USARTDIV)

	Mpart = USARTDIV;
	pUSARTx->BRR |= Mpart << 4;

	fraction = 16*(USARTDIV - Mpart);
	//The nearest real number of DIV_Fractıon
	Fpart = round(fraction);
	pUSARTx->BRR |= Fpart;
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Enable the clock for USARTx peripheral
	USART_PClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx Mode
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX){
		//transmitter enable
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX){
		//receiver enable
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RE);
	}
	else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		pUSARTHandle->pUSARTx->CR1 |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	//Determines the word length
	pUSARTHandle->pUSARTx->CR1 |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M;

	//Parity control and selection
	if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_EVEN){
		//Parity control enabled
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
		//even parity
		pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_ODD){
		//Parity control enabled
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PCE);
		//odd parity
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_PS);
	}

	//These bits are used for programming the stop bits
	pUSARTHandle->pUSARTx->CR2 |= pUSARTHandle->USART_Config.USART_StopBits << USART_CR2_STOP;

	//Configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_BaudRate);

}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();

	}else if(pUSARTx == USART2){
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3){
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4){
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5){
		UART5_REG_RESET();
	}
}


void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
		pUSARTx->CR1 |= (1<<USART_CR1_UE);
	}
	else{
		pUSARTx->CR1 &= ~(1<<USART_CR1_UE);
	}
}


uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
	if(pUSARTx->SR & FlagName){
		return FLAG_SET;
	}
	return FLAG_RESET;
}



void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxbuffer, uint32_t Len)
{
	for(uint32_t i=0; i<Len; i++){

	//wait until TXE bit is set by hardware when the content of the TDR register has been transferred into the shift register
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE));

	//check USART word length 8bit or 9bit
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_9BITS){

		//9bit load the DR with 2 byte
		pUSARTHandle->pUSARTx->DR |= ( *(uint16_t*)pTxbuffer & (uint16_t)0x01FF);

		//check parity bit
		if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
			//no parity 9bits of user data will be sent
			pTxbuffer++;
			pTxbuffer++;
		}
		else
			//parity bit used and 8bits of user data will be sent
			pTxbuffer++;
	}
	//8bit load the DR with 1 byte
	else{
		if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE)
		pUSARTHandle->pUSARTx->DR |= (*pTxbuffer & (uint8_t)0xFF);
		else{
			pUSARTHandle->pUSARTx->DR |= (*pTxbuffer & (uint8_t)0x7F);}

		pTxbuffer++;
	}
	//wait until TC is set by hardware if the transmission of a frame containing data is complete and if TXE is set
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC));

	}
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxbuffer, uint32_t Len)
{
	for(uint32_t i=0; i<Len; i++){

	//wait until RXNE is set by hardware when the content of the RDR shift register has been transferred into the USART_DR register
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));

	//check USART word length 8bit or 9bit
	if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_9BITS){

		//read 9bits from the DR with 2 byte
		(*(uint16_t*)pRxbuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

		//check parity bit
		if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
			//no parity 9bits of user data will be receive
			pRxbuffer++;
			pRxbuffer++;
		}
		else
			//parity bit used and 8bits of user data will be receive
			pRxbuffer++;
	}

	//read 8bit from DR
	else{
		if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE)
			*pRxbuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
		else{
			*pRxbuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);}

		pRxbuffer++;
	}
	}
}

USART_StatusTypeDef USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxbuffer, uint32_t Len)
{
	USART_StatusTypeDef StatusTx = pUSARTHandle->TxState;

	if(StatusTx != USART_BUSY_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxbuffer;
		pUSARTHandle->TxState = USART_BUSY_TX;
	}

	//TXE interrupt enable
	pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

	//Transmission complete interrupt enable
	pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);

	return StatusTx;
}

USART_StatusTypeDef USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxbuffer, uint32_t Len)
{
	USART_StatusTypeDef StatusRx = pUSARTHandle->RxState;

	if(StatusRx != USART_BUSY_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxbuffer;
		pUSARTHandle->RxState = USART_BUSY_RX;
	}

	//RXNE interrupt enable
	pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	return StatusRx;
}

void USART_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void USART_IRQPriorityConfig(uint32_t IRQPriority, uint8_t IRQNumber)
{
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;
	uint8_t shift_amount = (8 * IPRx_Section) + (8 - NO_IPR_BITS_IMPLEMENTED); //unimplemented low-order 4 bits IPR register
	*(NVIC_IPR + IPRx) |=  (IRQPriority << shift_amount);

}

static void USART_TXEHandleIT(USART_Handle_t *pUSARTHandle)
{
	if(pUSARTHandle->TxLen > 0  &&  pUSARTHandle->TxState == USART_BUSY_TX){
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_9BITS){
			//9bit load the DR with 2 byte
			pUSARTHandle->pUSARTx->DR |= ( *(uint16_t*)pUSARTHandle->pTxBuffer & (uint16_t)0x01FF);
			//check parity bit
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
						//no parity 9bits of user data will be sent
				pUSARTHandle->pTxBuffer+=2;
				pUSARTHandle->TxLen-=2;
			}
			else{
			//parity bit used and 8bits of user data will be sent
			pUSARTHandle->pTxBuffer++;
			pUSARTHandle->TxLen--;}
		}
		//8bit load the DR with 1 byte
		else{
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE)
				pUSARTHandle->pUSARTx->DR |= (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);
			else{
				pUSARTHandle->pUSARTx->DR |= (*pUSARTHandle->pTxBuffer & (uint8_t)0x7F);}

				pUSARTHandle->pTxBuffer++;
				pUSARTHandle->TxLen--;
			}
	}
	if(pUSARTHandle->TxLen == 0){
		//disable TXE flag clear TXEIE bit
		pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TXEIE);
		pUSARTHandle->TxState = USART_READY;
	}
}

static void USART_RXNEHandleIT(USART_Handle_t *pUSARTHandle)
{
	if(pUSARTHandle->RxLen > 0 && pUSARTHandle->RxState == USART_BUSY_RX){
		//check USART word length 8bit or 9bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORD_9BITS){

				//read 9bits from the DR with 2 byte
			(*(uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

				//check parity bit
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE){
					//no parity 9bits of user data will be receive
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen-=2;
			}
			else{
					//parity bit used and 8bits of user data will be receive
				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;}
			}

			//read 8bit from DR
		else{
			if(pUSARTHandle->USART_Config.USART_Parity == USART_PARITY_DISABLE)
				*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
			else{
				*(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);}

				pUSARTHandle->pRxBuffer++;
				pUSARTHandle->RxLen--;
			}
	}
	else{
		//disable RXNE flag clear RXNEIE bit
		pUSARTHandle->pUSARTx->CR1 &= ~( 1<< USART_CR1_RXNEIE);
		pUSARTHandle->RxState = USART_READY;
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX);
	}
}

void USART_EVENT_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1=0, temp2=0;

	//Check the interrupt event Transmit data TXE register empty
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2){
		//handle TXE interrupt
		USART_TXEHandleIT(pUSARTHandle);
	}

	//Check the interrupt event Received data ready to be read RXNE
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2){
		//handle RXNE interrupt
		USART_RXNEHandleIT(pUSARTHandle);
	}

	//Check the interrupt event transmission complete for TC flag and TCIE enable control bit
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2){
		if(pUSARTHandle->TxLen == 0 && pUSARTHandle->TxState == USART_BUSY_TX){

			//clear the TC flag and TCIE control bit
			pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);
			pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_TCIE);

			//reset state and buffer
			pUSARTHandle->TxState = USART_READY;
			pUSARTHandle->pTxBuffer = NULL;

			USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX);
		}
	}

	//Check the interrupt event CTS flag
	//CTS: Clear To Send blocks the data transmission at the end of the current transfer when high
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
	uint32_t temp3 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSIE);
	if(temp1 && temp2 && temp3){
		//clear CTS
		pUSARTHandle->pUSARTx->SR &=  ~( 1 << USART_SR_CTS);

		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

	//Check the interrupt event Overrun error detected
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
	if(temp1 && temp2 ){

		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

	//Check the interrupt event idle line detected
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_IDLE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);
	if(temp1 && temp2){
		//The IDLE bit will not be set again until the RXNE bit has been set itself
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

	//Check the interrupt event Break flag
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_LBD);
	temp2 = pUSARTHandle->pUSARTx->CR2 & (1 << USART_CR2_LBDIE);

	if(temp1 && temp2){

		//give for the application to select break detection length from CR2 register LBDL bit and to clear the LBD flag
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_BREAK);
	}

	//Check the interrupt event Parity error
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_PE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_PEIE);

	if(temp1 && temp2){

		//The software must wait for the RXNE flag to be set before clearing the PE bit
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE));
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_PE);

		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_PARITY);
		}

	//Noise flag, Overrun error and Framing error in multibuffer communication

	if( pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE) ) {

		if(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_NE)){

			 /*This bit is set by hardware when noise is detected on a received frame. It is cleared by a
			   software sequence (an read to the USART_SR register followed by a read to the USART_DR register).*/

			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NE);
		}

		if(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_FE)){

			/*This bit is set by hardware when a de-synchronization, excessive noise or a break character
		      is detected. It is cleared by a software sequence (an read to the USART_SR register
			  followed by a read to the USART_DR register).*/

			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_ORE)){

			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}

	}

}


void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEv)
{


}

