/*
 * i2c_driver.h
 *
 *  Created on: 2 May 2020
 *      Author: ERCAN
 */

#ifndef INC_I2C_DRIVER_H_
#define INC_I2C_DRIVER_H_

#include <stm32f103xx.h>

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint8_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; 	/*!< To store the app. Tx buffer address 	>*/
	uint8_t 		*pRxBuffer;		/*!< To store the app. Rx buffer address 	>*/
	uint32_t 		TxLen;			/*!< To store Tx len 						>*/
	uint32_t 		RxLen;			/*!< To store Rx len 						>*/
	uint8_t 		TxRxState;		/*!< To store Communication state 			>*/
	uint8_t 		DevAddr;		/*!< To store slave/device address 			>*/
	uint8_t     	ReStart;		/*!< To store repeated start value 			>*/
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 */
#define	I2C_SCL_SPEED_SM1K		100000
#define	I2C_SCL_SPEED_FM2K		200000
#define	I2C_SCL_SPEED_FM4K		400000

// @I2C_DeviceAddress this value will be mentioned by the user

/*
 * @I2C_AckConrtol
 */
#define I2C_ACK_EN				1
#define I2C_ACK_DI				0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   		( 1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	( 1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			( 1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		( 1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		( 1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		( 1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		( 1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		( 1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		( 1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		( 1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		( 1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	( 1 << I2C_SR1_TIMEOUT)

#define I2C_REPEATED_START_DI	RESET
#define I2C_REPEATED_START_EN	SET

/*
 * I2C application events
 */
#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_RECEIVE			8
#define I2C_EV_REQUEST			9

/*
 * I2Cx states
 */

typedef enum{
	I2C_READY = 0,
	I2C_BUSY_RX = 1,
	I2C_BUSY_TX = 2
}I2C_StatusTypeDef;

/*************************************************************************************
 * 							APIs supported by this driver
 *								Functions Prototypes
 ************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void I2C_PClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
  * İnitializing and De-initializing
  */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 * Send and Receive Data
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t ReStart);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t ReStart);
I2C_StatusTypeDef I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t ReStart);
I2C_StatusTypeDef I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t ReStart);

void I2C_SlaveSendData(I2C_Handle_t *pI2CHandle, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_Handle_t *pI2CHandle);


/*
 * IRQ (interrupt) Configuration and ISR Handling
 */
void I2C_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint32_t IRQPriority, uint8_t IRQNumber);
void I2C_EVENT_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ERROR_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * other function type for Peripheral Control
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_SlaveCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);
void I2C_ManageACK(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_I2C_DRIVER_H_ */
