/*
 * stm32f103xx_rcc_driver.h
 *
 *  Created on: 13 May 2020
 *      Author: ERCAN
 */

#ifndef INC_RCC_DRIVER_H_
#define INC_RCC_DRIVER_H_

#include<stm32f103xx.h>

uint32_t RCC_PCLK1_GetValue(void);
uint32_t RCC_PCLK2_GetValue(void);

//uint32_t RCC_PLLOutputClock(void);

#endif /* INC_RCC_DRIVER_H_ */
