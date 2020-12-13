/*
 * stm32f103xx_gpio_driver.c
 *
 *  Created on: Apr 14, 2020
 *      Author: ERCAN
 */

#include <gpio_driver.h>


void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
	}
		else if (EnorDi == DISABLE)
		{

		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();

		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}

		}
	}

 /*
  * Initializing and De-Initializing
  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	GPIO_PClockControl(pGPIOHandle->pGPIOx, ENABLE);

	uint8_t temp1 = 0;
	uint8_t temp2 = 0;


	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_OUTPUT_SPEED_50M){

		// the non interrupt mode

		//Configure Input Mode
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->CR[temp1] &= ~(3 << (4 * temp2)); //clear MODEx
		pGPIOHandle->pGPIOx->CR[temp1] &= ~(3 << (4 * (temp2 >> 2))); //clear CNFx
		pGPIOHandle->pGPIOx->CR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_InputMode << (4 * (temp2 >> 2) ));

		temp1 = 0;
		temp2 = 0;

	}

	else
	{	//interrupt mode

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){

			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO selection in AFIO_EXTICR
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		AFIO_PCLK_EN();
		AFIO->EXTICR[temp1] |= portcode << (temp2 * 4);

		// enable EXTI Interrupt using IMR Register
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

		temp1 = 0;
		temp2 = 0;

		//Configure Output Mode
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->CR[temp1] &= ~(3 << (4 * (temp2 >> 2)));
		pGPIOHandle->pGPIOx->CR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_OutputMode << (4 * (temp2 >> 2)));

		temp1 = 0;
		temp2 = 0;

		//Configure Speed
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->CR[temp1] &= ~(3 << (4 * temp2));
		pGPIOHandle->pGPIOx->CR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (4 * temp2));

		if(pGPIOHandle->GPIO_PinConfig.GPIO_OutputMode >= GPIO_MODE_ALT_PP )
		{
			AFIO->EVCR |= ( (GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx) << 4) ); //select port

			AFIO->EVCR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber << 0); //select pin
		}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOE_REG_RESET();
	}
}


/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET ){
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{

	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1<<PinNumber);
}


/*
 * IRQ (Interrupt) Configuration and ISR Handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}

}

void GPIO_IRQPriorityConfig(uint32_t IRQPriority, uint8_t IRQNumber)
{
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;
	uint8_t shift_amount = (8 * IPRx_Section) +(8 - NO_IPR_BITS_IMPLEMENTED); //unimplemented low-order 4 bits IPR register
	*(NVIC_IPR + IPRx) |=  (IRQPriority << shift_amount);

}


void GPIO_IRQHandling(uint8_t PinNumber)
{
	/*
	 * 	Clear the PR (Pending Register) corresponding to the pin number
	 *  İf you don't do that you you will get interrupt again
	 */
	if(EXTI->PR & (1 << PinNumber)){

		//clear
		EXTI->PR |= (1 << PinNumber);
	}

}

