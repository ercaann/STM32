/*
 * spi_slave.c
 *
 *  Created on: 29 Nis 2020
 *      Author: ERCAN
 */

/*
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 -> SPI1_SCLK
 * PA4 --> SPI1_NSS
 */

#include "stm32f103xx.h"
#include "spi_driver.h"
#include<string.h>

#define LED_CONTROL			0x10
#define PRINT				0x12
#define ID_READ				0x13

#define LED_ON				1
#define LED_OFF				0
#define LED_PIN				GPIO_PIN_NO_13

#define ACK					0xAA
#define NACK				0xA0

uint8_t board_id[10]="STM32F103x";
char data[255];


void SPI1_GPIO_SlaveInit(void){

	GPIO_Handle_t SPISlave;
	SPISlave.pGPIOx = GPIOA;
	SPISlave.GPIO_PinConfig.GPIO_OutputMode = GPIO_MODE_ALT_PP;
	SPISlave.GPIO_PinConfig.GPIO_InputMode = GPIO_INPUT_PUPD;
	SPISlave.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUTPUT_SPEED_50M;

	SPISlave.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;  // MOSI
	GPIO_Init(&SPISlave);

	SPISlave.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;  // MISO
	GPIO_Init(&SPISlave);

	SPISlave.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;  // SCLK
	GPIO_Init(&SPISlave);

	SPISlave.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;  // NSS
	GPIO_Init(&SPISlave);

}

void SPI1_SlaveInit(void){

	SPI_Handle_t SPISlavehandle;
	SPISlavehandle.pSPIx = SPI1;
	SPISlavehandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPISlavehandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_SLAVE;
	SPISlavehandle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;
	SPISlavehandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPISlavehandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPISlavehandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPISlavehandle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPISlavehandle);
}

uint8_t CheckData(uint8_t command){

	return ACK;
}

int main()
{
	SPI1_GPIO_SlaveInit();
	SPI1_SlaveInit();
	SPI_PeripheralControl(SPI1, ENABLE);

	uint8_t command, dummy, ackornack=NACK;

	uint32_t i=0;

	while( 1 ) {  // generated until SS LOW


	SPI_ReceiveData(SPI1, &command, 1);

	ackornack = CheckData(command);

	SPI_SendData(SPI1, &ackornack, 1);

	SPI_ReceiveData(SPI1, &dummy, 1);


	if(command == LED_CONTROL){

		uint8_t state[2];

		SPI_ReceiveData(SPI1, state, 2);

		if(state[0] == (uint8_t)LED_ON){

			GPIO_WriteToOutputPin(GPIOC, state[1], 1);
		}

		else if(state[0] == (uint8_t)LED_OFF){

			GPIO_WriteToOutputPin(GPIOC, state[1], 0);
		}
	}

	else if(command == PRINT){

	uint8_t len = strlen(data);
	SPI_ReceiveData(SPI1, &len , 1);

	for(i=0; i<len; i++){

		SPI_ReceiveData(SPI1, (uint8_t*)data, strlen(data));
		//print
	}

	}

	else if(command == ID_READ){

		for(i=0; i < strlen((char*)board_id); i++){

			SPI_SendData(SPI1, &board_id[i], 1);
		}

	}
	}
	return 0;
}

