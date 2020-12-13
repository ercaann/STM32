/*
 * led_toggle.c
 *
 *  Created on: 17 Nis 2020
 *      Author: ERCAN
 */

#include "gpio_driver.h"



int main(){

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT_SPEED_50M;
	GpioLed.GPIO_PinConfig.GPIO_OutputMode = GPIO_OUTYPE_PP;


	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_13);
		delay();

	}
	return 0;
}

void delay(void)
{

	for(uint32_t i=0; i<50000; i++);
}
