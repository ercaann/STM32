/*
 * spi_rx.c
 *
 *  Created on: 30 Nis 2020
 *      Author: ERCAN
 */

/*
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 -> SPI1_SCLK
 * PA4 --> SPI1_NSS
 */

#include "stm32f103xx.h"
#include <string.h>

char data[500];


void SPI1_GPIO_SlaveInit(void){

	GPIO_Handle_t SPISlave;
	SPISlave.pGPIOx = GPIOA;
	SPISlave.GPIO_PinConfig.GPIO_OutputMode = GPIO_MODE_ALT_PP;
	SPISlave.GPIO_PinConfig.GPIO_InputMode = GPIO_INPUT_PUPD;
	SPISlave.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUTPUT_SPEED_50M;

	SPISlave.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;  //MOSI
	GPIO_Init(&SPISlave);

	//SPISlave.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;  //MISO
	//GPIO_Init(&SPISlave);

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
	SPISlavehandle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV8;
	SPISlavehandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPISlavehandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPISlavehandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPISlavehandle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPISlavehandle);
}

int main(void){

	SPI1_GPIO_SlaveInit();
	SPI1_SlaveInit();
	SPI_PeripheralControl(SPI1, ENABLE);


	uint8_t data_len= strlen(data);
	uint32_t i=0;

	while(1){

		SPI_ReceiveData(SPI1, &data_len, strlen(data));

		for(i=0; i<data_len; i++){

			SPI_ReceiveData(SPI1, (uint8_t*)data, strlen(data));

		}
	}

}
