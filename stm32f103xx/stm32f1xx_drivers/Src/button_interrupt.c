/*
 * led_toggle.c
 *
 *  Created on: 17 Nis 2020
 *      Author: ERCAN
 */
#include "gpio_driver.h"

#include<string.h>

#define HIGH	1
#define LOW		0
#define BUTTON_PRESS	LOW



int main(){

	GPIO_Handle_t GpioLed, GpioButton;

    // Initialize each and every member element of these structure to 0 (to avoid corrupted register)
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioButton, 0, sizeof(GpioButton));

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_OutputMode = GPIO_OUTYPE_PP ;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_SPEED_50M;

	GPIO_Init(&GpioLed);

	GpioButton.pGPIOx = GPIOB;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
	GpioButton.GPIO_PinConfig.GPIO_InputMode=GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT_SPEED_50M;
	GpioButton.GPIO_PinConfig.GPIO_InputMode=GPIO_INPUT_PUPD;

	GPIO_Init(&GpioButton);

	GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NO_13, RESET);

	//IRQ Configuration
	GPIO_IRQITConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(NVIC_IRQ_PRI15, IRQ_NO_EXTI9_5);

	while(1);

	return 0;
}

void delay(void)
{

	for(uint32_t i=0; i<50000; i++);
}

void EXTI9_5_IRQHandler(void){

	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_5); //clear the pending event from EXTI line
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
}
