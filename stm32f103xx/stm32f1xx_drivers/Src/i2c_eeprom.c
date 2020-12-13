/*
 * i2c_eeprom.c
 *
 *  Created on: 2 May 2020
 *      Author: ERCAN
 */


/*
 * 32Kbits EEPROM has 4096 addresses
 * 0x1000. Since the memory locations starts from 0, then the last one has the 0x0FFF address.
 */


#include<stm32f103xx.h>
#include<stdlib.h>
#include<string.h>

#define SLAVE_ADDR		0xA0
#define OWN_ADDR		0x61
#define MemAddres		0x0ABC

I2C_Handle_t I2C1Handle;
GPIO_Handle_t I2CPins;

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

void Write_To_24LCxx(I2C_Handle_t *pI2Cx, uint16_t SlaveAddr, uint16_t MemAddr, uint8_t *pData, uint16_t Len)
{

	uint8_t *data;

	//allocate memory for address and data to store
	data = (uint8_t*)malloc(sizeof((uint8_t*)(Len+2)));

	data[0] = (uint8_t) ((MemAddr & 0xFF00) >> 8);
	data[1] = (uint8_t) (MemAddr & 0xFF);

	//copy the content of the pData array in the temporary buffer
	memcpy(data+2, pData, Len);

	//send data buffer
	I2C_MasterSendData(&I2C1Handle, data, Len+2, SlaveAddr, 0);

	//free up previously allocated memory
	free(data);

}

void Read_From_24LCxx(I2C_Handle_t *pI2Cx, uint16_t SlaveAddr, uint16_t MemAddr, uint8_t *pData, uint16_t Len)
{
	uint8_t addr[2];

	//compute the MSB and LSB parts of the memory address (0000-3FFF)
	addr[0] = (uint8_t) ((MemAddr & 0xFF00) >> 8);
	addr[1] = (uint8_t) (MemAddr & 0xFF);

	//send the memory location address where start reading data
	I2C_MasterSendData(&I2C1Handle, addr, 2, SlaveAddr, 1);

	//can retrieve the data from EEPROM
	I2C_MasterReceiveData(&I2C1Handle, pData, Len, SlaveAddr, 0);

}

int main(void){

	I2C1_GPIOInits();

	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageACK(I2C1, I2C_ACK_EN);

	const char write_msg[] = "I2C EEPROM Read/Write";
	char read_msg[30];

	while(1){

	delay();

	Write_To_24LCxx(&I2C1Handle, SLAVE_ADDR, MemAddres, (uint8_t*)write_msg, strlen(write_msg)+1);
	Read_From_24LCxx(&I2C1Handle, SLAVE_ADDR, MemAddres, (uint8_t*)read_msg, strlen(read_msg)+1);

	}
}
