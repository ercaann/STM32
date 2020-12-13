/*
 * spi_tx.c
 *
 *  Created on: 25 Nis 2020
 *      Author: ERCAN
 */
#include <string.h>
#include "stm32f103xx.h"
#include "spi_driver.h"

/*
 * PA6 --> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * PA5 -> SPI1_SCLK
 * PA4 --> SPI1_NSS
 */




void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_OutputMode = GPIO_MODE_ALT_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUTPUT_SPEED_50M;
	SPIPins.GPIO_PinConfig.GPIO_InputMode = GPIO_INPUT_PUPD;

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;  //SCLK
	GPIO_Init(&SPIPins);

	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;  //MOSI
	GPIO_Init(&SPIPins);


	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;  //MISO
	//GPIO_Init(&SPIPins);

	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;  //NSS
	//GPIO_Init(&SPIPins);
}

void SPI1_Inits(void){

	SPI_Handle_t SPI1handle;


	SPI1handle.pSPIx = SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI1handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI1handle.SPIConfig.SPI_CPHA = SPI_CPHA_HIGH;
	SPI1handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI1handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI1handle);
}

int main()
{
	char data[] = "Embedded System";

	SPI1_GPIOInits();
	SPI1_Inits();

	//this makes NSS signal internally high and avoids MODF error
	SPI_SSIConfig(SPI1, ENABLE);

	//enable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, ENABLE);

	//send data
	SPI_SendData(SPI1, (uint8_t*)data, strlen(data));

	//SPI (or I2S) is busy in communication or Tx buffer is not empty
	while(SPI_GetFlagStatus(SPI1,SPI_FLAG_BUSY));

	//disable the SPI1 peripheral
	SPI_PeripheralControl(SPI1, DISABLE);

	while(1);

	return 0;
}

