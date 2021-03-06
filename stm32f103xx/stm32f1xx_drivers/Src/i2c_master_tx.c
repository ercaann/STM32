/*
 * i2c_master_tx.c
 *
 *  Created on: 1 May 2020
 *      Author: ERCAN
 */

#include<stdio.h>
#include<string.h>
#include "stm32f103xx.h"
#include "i2c_driver.h"

 /*
  * PB6 --> I2C1_SCL
  * PB7 --> I2C1_SDA
  */


#define SLAVE_ADDR		0x68
#define OWN_ADDR		0x61

I2C_Handle_t I2C1Handle;

uint8_t data[30]="I2C Master Tx test\n";

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

void I2C1_GPIOInits(void)
{

	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_OutputMode = GPIO_MODE_ALT_OD;
	I2CPins.GPIO_PinConfig.GPIO_InputMode = GPIO_INPUT_PUPD;  //need to connect external pull up
	I2CPins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_OUTPUT_SPEED_50M;

	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6; // SCL
	GPIO_Init(&I2CPins);

	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7; // SDA
	GPIO_Init(&I2CPins);
}


void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_EN;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = OWN_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM1K;

	I2C_Init(&I2C1Handle);

}

int main(void){

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageACK(I2C1, ENABLE);

	I2C1_GPIOInits();

	I2C1_Inits();

	while(1){

		delay();

		I2C_MasterSendData(&I2C1Handle, data, strlen((char*)data), SLAVE_ADDR, 0);
	}

}
