/*
 * stm32f103xx_rcc_driver.c
 *
 *  Created on: 13 May 2020
 *      Author: ERCAN
 */

#include <rcc_driver.h>

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_Prescaler[4] = {2,4,8,16};

//uint32_t RCC_PLLOutputClock(void){}

uint32_t RCC_PCLK1_GetValue(void)
{
	uint32_t PCLK1,SYSCLK;
	uint8_t temp=0,clk=0,AHB,APB1;
	clk = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);

	if (clk == 0){
		SYSCLK = 8000000; // HSI 8MHz
	}
	else if(clk == 1){
		SYSCLK = 4000000;  // HSE 4MHz
	}
	else if(clk == 2){
		//SYSCLK == RCC_PLLOutputClock();
	}

	temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
	if(temp < 8){
		AHB=1;
	}
	else{
		AHB = AHB_Prescaler[temp - 8];
	}

	temp = ((RCC->CFGR >> RCC_CFGR_PPRE1) & 0x7);
	if(temp < 4){
			APB1=1;
	}
	else{
			APB1 = APB_Prescaler[temp - 4];
	}

	PCLK1 = (SYSCLK / AHB) / APB1;

	return PCLK1;
}

uint32_t RCC_PCLK2_GetValue(void)
{
	uint32_t PCLK2,SYSCLK=0;
	uint8_t temp=0,clk=0,AHB,APB2;
	clk = ((RCC->CFGR >> RCC_CFGR_SWS) & 0x3);

	if (clk == 0){
		SYSCLK = 8000000; // HSI 8MHz
	}
	else if(clk == 1){
		SYSCLK = 4000000;  // HSE 4MHz
	}
	else if(clk == 2){
		//SYSCLK == RCC_PLLOutputClock();
	}

	temp = ((RCC->CFGR >> RCC_CFGR_HPRE) & 0xF);
	if(temp < 8){
		AHB=1;
	}
	else{
		AHB = AHB_Prescaler[temp - 8];
	}

	temp = ((RCC->CFGR >> RCC_CFGR_PPRE2) & 0x7);
	if(temp < 4){
			APB2=1;
	}
	else{
			APB2 = APB_Prescaler[temp - 4];
	}

	PCLK2 = (SYSCLK / AHB) / APB2;

	return PCLK2;
}
