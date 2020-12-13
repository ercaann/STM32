/*
 * i2c_slave_tx.c
 *
 *  Created on: 30 Nis 2020
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
#define OWN_ADDR		SLAVE_ADDR


uint8_t message[32]="I2C Slave Tx test";
uint8_t command=0xff, len;
uint8_t cnt=0;

I2C_Handle_t I2C1Handle;

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


	I2C1_GPIOInits();

	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQITConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQITConfig(IRQ_NO_I2C1_ER, ENABLE);


	I2C_SlaveCallbackEvents(I2C1,ENABLE);

	while(1);

}

void I2C1_EV_IRQHandler (void)
{
	I2C_EVENT_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler (void){

	I2C_ERROR_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv){

	if(AppEv == I2C_EV_REQUEST){

		if(command == 0x11){

			I2C_SlaveSendData(&I2C1Handle, strlen((char*)message));
		}

		else if(command == 0x12){

			I2C_SlaveSendData(&I2C1Handle, message[cnt++]);
		}
	}
	else if(AppEv == I2C_EV_RECEIVE){

		command = I2C_SlaveReceiveData(&I2C1Handle);
	}

	else if(AppEv == I2C_ERROR_AF){
		// master send NACK. master doesn't need more data
		command = 0xff;
		cnt = 0;
	}
	else if(AppEv == I2C_EV_STOP){
		//only slave
	}
}
