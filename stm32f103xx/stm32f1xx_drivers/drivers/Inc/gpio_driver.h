/*
 * stm32f103xx_gpio_driver.h
 *
 *  Created on: Apr 14, 2020
 *      Author: ERCAN
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f103xx.h"


/*
 * Configuration Structure for GPIO Pin
 */
typedef struct
{

	uint8_t GPIO_PinMode;					/*!< 	@GPIO_Pin_Modes 	>*/
	uint8_t GPIO_PinNumber;					/*!<  	@GPIO_Pin_Numbers 	>*/
	uint8_t GPIO_OutputMode;				/*!< 	@GPIO_Output_Mode	>*/
	uint8_t GPIO_InputMode;					/*!< 	@GPIO_Input_Mode	>*/
	uint8_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;


/*
 * Handle of Structure for GPIO and AFIO Pin
 */
typedef struct
{

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;


/*
 * 	@GPIO_Pin_Modes
 */
#define GPIO_MODE_INPUT				0
#define GPIO_MODE_OUTPUT_SPEED_10M	1
#define GPIO_MODE_OUTPUT_SPEED_2M	2
#define GPIO_MODE_OUTPUT_SPEED_50M	3
#define GPIO_MODE_IT_FT				4
#define GPIO_MODE_IT_RT				5
#define GPIO_MODE_IT_RFT			6


/*
 * @GPIO_Output_Mode_Type
 */
#define GPIO_OUTYPE_PP				0
#define GPIO_OUTYPE_OD				1
#define GPIO_MODE_ALT_PP			2
#define GPIO_MODE_ALT_OD			3

/*
 * @GPIO_Input_Mode
 */
#define GPIO_INPUT_ANALOG			0
#define GPIO_INPUT_FLOATING			1
#define GPIO_INPUT_PUPD				2
#define GPIO_INPUT_RESERVED			3


/*
 * @GPIO_Pin_Numbers
 */
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15


/*************************************************************************************
 * 						APIs supported by this driver
 *							Functions Prototypes
 ************************************************************************************/

/*
 * Peripheral Clock Setup
 */
void GPIO_PClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
void AFIIO_PClockControl(AFIO_RegDef_t *pPx, uint8_t EnorDi);

 /*
  * Initializing and De-initializing
  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ (interrupt) Configuration and ISR Handling
 */
void GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint32_t IRQPriority, uint8_t IRQNumber);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_GPIO_DRIVER_H_ */
