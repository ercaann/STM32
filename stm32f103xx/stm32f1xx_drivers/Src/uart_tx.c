/*
 * uart_tx.c
 *
 *  Created on: 3 May 2020
 *      Author: ERCAN
 */

#include<stdio.h>
#include<string.h>
#include"stm32f103xx.h"

char message[1000]="UART Tx testing\n\r";

USART_Handle_t USART1Handle;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void USART1_Inits(void){

	USART1Handle.pUSARTx = USART1;
	USART1Handle.USART_Config.USART_BaudRate = USART_BAUDRATE_115200;
	USART1Handle.USART_Config.USART_HwFlowCtl = USART_HWFLOWCTL_NONE;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_TX;
	USART1Handle.USART_Config.USART_Parity = USART_PARITY_DISABLE;
	USART1Handle.USART_Config.USART_StopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_WordLength = USART_WORD_8BITS;

	USART_Init(&USART1Handle);
}

void USART1_GPIOInit(void){

	GPIO_Handle_t USART1Pin;

	USART1Pin.pGPIOx = GPIOA;

	USART1Pin.GPIO_PinConfig.GPIO_OutputMode = GPIO_MODE_ALT_PP;
	USART1Pin.GPIO_PinConfig.GPIO_InputMode = GPIO_INPUT_PUPD;
	USART1Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_SPEED_50M;

	//USART Tx
	USART1Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&USART1Pin);

	//USART Rx
	USART1Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&USART1Pin);

}
int main(void){

	USART1_GPIOInit();

	    USART1_Inits();

	    USART_PeripheralControl(USART1, ENABLE);

	    while(1)
	    {

			USART_SendData(&USART1Handle, (uint8_t*)message, strlen(message));

	    }

		return 0;
}
