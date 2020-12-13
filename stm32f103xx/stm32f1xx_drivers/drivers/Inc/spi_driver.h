/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: 24 Nis 2020
 *      Author: ERCAN
 */

#ifndef INC_SPI_DRIVER_H_
#define INC_SPI_DRIVER_H_

#include "stm32f103xx.h"


/*
 * Configuration Structure for SPIx Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;			/*!<  	@SPI_Decice_Mode	>*/
	uint8_t SPI_BusConfig;			/*!<  	@SPI_BusConfig 		>*/
	uint8_t SPI_SClkSpeed;			/*!<  	@SPI_SClkSpeed 		>*/
	uint8_t SPI_DFF;				/*!<  	@SPI_DFF 			>*/
	uint8_t SPI_CPOL;				/*!<  	@SPI_CPOL 			>*/
	uint8_t SPI_CPHA;				/*!<  	@SPI_CPOL 			>*/
	uint8_t SPI_SSM;				/*!< 	@SPI_SSM 			>*/
}SPI_Config_t;


/*
 * Handle of Structure for SPIx Peripheral
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;			/*!<  hold the base address of SPIx(x:0,1,2) Peripheral >*/
	SPI_Config_t	SPIConfig;		/*!<  hold SPIx peripheral configuration settings 		>*/
	uint8_t			*pTxBuffer;		/*!<  To store Tx buffer address >*/
	uint8_t			*pRxBuffer;		/*!<  To store Rx buffer address >*/
	uint32_t 		TxLen;			/*!<  To store Tx length 		 >*/
	uint32_t 		RxLen;			/*!<  To store Rx length 		 >*/
	uint8_t			TxState;		/*!<  To store Tx store 		 >*/
	uint8_t			RxState;		/*!<  To store Rx store 		 >*/

}SPI_Handle_t;

/*
 * @SPI_Decice_Mode
 */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD			1  		// FULL_DUPLEX
#define SPI_BUS_CONFIG_HD			2		// HALF_DUPLEX
#define SPI_BUS_CONFIG_SRX			3		// SIMPLEX_RXONLY

/*
 * @SPI_SClkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW				0		// CK to 0 when idle
#define SPI_CPOL_HIGH				1		// CK to 1 when idle

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW				0		// The first clock transition is the first data capture edge
#define SPI_CPHA_HIGH				1		// The second clock transition is the first data capture edge

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI					0		// Software slave management disabled
#define SPI_SSM_EN					1		// Software slave management enabled

/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   		1
#define SPI_EVENT_RX_CMPLT   		2
#define SPI_EVENT_OVR_ERR    		3
#define SPI_EVENT_CRC_ERR    		4

/*
 * SPI related status flags definitions
 */
#define SPI_FLAG_TXE    ( 1 << SPI_SR_TXE)
#define SPI_FLAG_RXNE   ( 1 << SPI_SR_RXNE)
#define SPI_FLAG_BUSY   ( 1 << SPI_SR_BSY)


/*************************************************************************************
 * 							APIs supported by this driver
 *								Functions Prototypes
 ************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void SPI_PClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);


 /*
  * İnitializing and De-initializing
  */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len );
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len );

/*
 * IRQ (interrupt) Configuration and ISR Handling
 */
void SPI_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint32_t IRQPriority, uint8_t IRQNumber);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * other function type for Peripheral Control
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_CheckSS(SPI_RegDef_t *pSPIx);

uint8_t SPI_SendDataIT(SPI_Handle_t *pHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pHandle, uint8_t *pRxBuffer, uint32_t Len);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_SPI_DRIVER_H_ */
