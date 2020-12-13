/*
 * i2c_master_rxIT.c
 *
 *  Created on: 2 May 2020
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

uint8_t data[30];

uint8_t Rx_Cmplt = RESET;

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

	uint8_t command, len;

	I2C1_GPIOInits();

	I2C1_Inits();

	I2C_IRQITConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQITConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageACK(I2C1, ENABLE);

	while(1){

		delay();

		command = 0x11;

		while(I2C_MasterSendDataIT(&I2C1Handle, &command, 1, SLAVE_ADDR, 1) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, 1) != I2C_READY);

		command = 0x12;

		while(I2C_MasterSendDataIT(&I2C1Handle, &command, 1, SLAVE_ADDR, 1) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1Handle, data, 1, SLAVE_ADDR, 1) != I2C_READY);

		Rx_Cmplt = RESET;

		while(Rx_Cmplt != SET){
			//wait till rx completes
		}

		data[len+1]='\0';

		printf("data : %s",data);

		Rx_Cmplt = RESET;
	}

}

void I2C1_EV_IRQHandler (void)
{
	I2C_EVENT_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler (void){

	I2C_ERROR_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
     if(AppEv == I2C_EV_TX_CMPLT)
     {
    	 printf("Tx is completed\n");
     }else if (AppEv == I2C_EV_RX_CMPLT)
     {
    	 printf("Rx is completed\n");
    	 Rx_Cmplt = SET;
     }
     else if(AppEv == I2C_ERROR_AF){

    	 I2C_CloseSendData(&I2C1Handle);

    	 //stop
    	 pI2CHandle->pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
     }

     while(1);
}
