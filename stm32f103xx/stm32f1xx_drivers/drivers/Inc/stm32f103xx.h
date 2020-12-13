/*
 * stm32f103xx.h
 *
 *  Created on: Apr 14, 2020
 *      Author: ERCAN
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#include<stdint.h>
#include<stddef.h>
#include<math.h>

#define _vo volatile
#define __weak __attribute__((weak))


/********************************** Processor Specific Details **********************************/

/*
 * ARM CORTEX M3 Processor NVIC ISERx (İnterrupt Set-enable) Registers Addresses
 */
#define NVIC_ISER0		((_vo uint32_t*) 0xE000E100)
#define NVIC_ISER1		((_vo uint32_t*) 0xE000E104)
#define NVIC_ISER2		((_vo uint32_t*) 0xE000E108)

/*
 * ARM CORTEX M3 Processor NVIC ICERx (İnterrupt Clear-enable) Registers Addresses
 */
#define NVIC_ICER0		((_vo uint32_t*) 0xE000E180)
#define NVIC_ICER1		((_vo uint32_t*) 0xE000E184)
#define NVIC_ICER2		((_vo uint32_t*) 0xE000E188)


/*
 * ARM CORTEX M3 Processor NVIC IPR (İnterrupt Priority) Registers Addresses
 * unimplemented low-order (4)bits read as zero and ignore writes in IPR Register
 */
#define NVIC_IPR		((_vo uint32_t*) 0xE000E400)
#define NO_IPR_BITS_IMPLEMENTED		4


/*
 * Base Address of Flash and SRAM Memories
 */
#define FLASH_BASEADDR					0x08000000U
#define SRAM_BASEADDR					0x20000000U
#define ROM_BASEADRR					0x1FFFF000U


/*
 * AHBx and APBx Bus Peripheral Base Address
 */
#define PERIPH_BASEADDR					0x40000000U
#define APB1PERIPH_BASE					PERIPH_BASEADDR
#define APB2PERIPH_BASE					0x40010000U
#define AHBPERIPH_BASE					0x40018000U


/*
 * Base Address of Peripherals 	which are hang on APB2
 */
#define GPIOA_BASEADDR				(APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR				(APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR				(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR				(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR				(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR				(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR				(APB2PERIPH_BASE + 0x2000)

#define AFIO_BASEADDR				 APB2PERIPH_BASE
#define EXTI_BASEADDR				(APB2PERIPH_BASE + 0x0400)
#define SPI1_BASEADDR				(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR				(APB2PERIPH_BASE + 0x3800)


/*
 * Base Address of Peripherals 	which are hang on APB1
 */
#define I2C1_BASEADDR				(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASE + 0x5800)

#define SPI2_BASEADDR				(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR				(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASE + 0x5000)



/*
 *  Base Address of Peripherals which are hang on AHB
 */
#define RCC_BASEADDR				(AHBPERIPH_BASE	+ 0x9000)

#define DMA1_BASEADDR				(AHBPERIPH_BASE	+ 0x8000)
#define DMA2_BASEADDR				(AHBPERIPH_BASE	+ 0x8400)



/************************************************* Peripheral Register Definition Structures ************************************/

 /*
  * Peripheral Register Definition Structure for GPIO
  */
typedef struct
{
	_vo uint32_t CR[2];			  // CR[0] : Configuration Register Low, CR[1] : Configuration Register High
	_vo uint32_t IDR;			  // İnput Data Register
	_vo uint32_t ODR;			  // Output Data Register
	_vo uint32_t BRR;			  // Bit Reset Register
	_vo uint32_t LCKR;			  // Configuration Lock Register
}GPIO_RegDef_t;


/*
 * Peripheral Register Definition Structure for AFIO
 */
typedef struct
{
	_vo uint32_t EVCR;
	_vo uint32_t MAPR;
	_vo uint32_t EXTICR[3];
}AFIO_RegDef_t;


/*
 * Peripheral Register Definition Structure for RCC
 */
typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t APB2RSTR;
	_vo uint32_t APB1RSTR;
	_vo uint32_t AHBENR;
	_vo uint32_t APB2ENR;
	_vo uint32_t APB1ENR;
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	_vo uint32_t AHBSTR;
	_vo uint32_t CFGR2;
}RCC_RegDef_t;


/*
 * Peripheral Register Definition Structure for EXTI
 *
 */
typedef struct
{
	_vo uint32_t IMR;
	_vo uint32_t EMR;
	_vo uint32_t RTSR;
	_vo uint32_t FTSR;
	_vo uint32_t SWIER;
	_vo uint32_t PR;
}EXTI_RegDef_t;


/*
 * Peripheral Register Definition Structure for SPIx
 *
 */

typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t CRCPR;
	_vo uint32_t RXCRCR;
	_vo uint32_t TXCRCR;
	_vo uint32_t I2SCFGR;
	_vo uint32_t I2SPR;
}SPI_RegDef_t;

/*
 * Peripheral Register Definition Structure for I2Cx
 */
typedef struct
{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t OAR1;
	_vo uint32_t OAR2;
	_vo uint32_t DR;
	_vo uint32_t SR1;
	_vo uint32_t SR2;
	_vo uint32_t CCR;
	_vo uint32_t TRISE;
	_vo uint32_t FLTR;
}I2C_RegDef_t;

/*
 * Peripheral Register Definition Structure for USARTx
 */
typedef struct
{
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t BRR;
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t CR3;
	_vo uint32_t GTPR;
}USART_RegDef_t;


typedef enum
{
	OK			=0x01,
	ERROR		=0x00
}StatusTypeDef;


/******************************************************************************************
 *Bit position definitions of RCC
 ******************************************************************************************/

/*
 * Bit position definitions RCC_CFGR
 */
#define RCC_CFGR_SW						0
#define RCC_CFGR_SWS					2
#define RCC_CFGR_HPRE					4
#define RCC_CFGR_PPRE1					8
#define RCC_CFGR_PPRE2					11
#define RCC_CFGR_ADCPRE					14
#define RCC_CFGR_PLLSRC					16
#define RCC_CFGR_PLLXTPRE				17
#define RCC_CFGR_PLLMUL					18
#define RCC_CFGR_MCO					24

/*
 * Bit position definitions RCC_APB1ENR
 */
#define RCC_APB1ENR_TIM2EN				0
#define RCC_APB1ENR_TIM3EN				1
#define RCC_APB1ENR_TIM4EN				2
#define RCC_APB1ENR_TIM5EN				3
#define RCC_APB1ENR_TIM6EN				4
#define RCC_APB1ENR_TIM7EN				5
#define RCC_APB1ENR_WWDGEN				11
#define RCC_APB1ENR_SPI2EN				14
#define RCC_APB1ENR_SPI3EN				15
#define RCC_APB1ENR_USART2EN			17
#define RCC_APB1ENR_USART3EN			18
#define RCC_APB1ENR_UART4EN				19
#define RCC_APB1ENR_UART5EN				20
#define RCC_APB1ENR_I2C1EN				21
#define RCC_APB1ENR_I2C2EN				22
#define RCC_APB1ENR_CAN1EN				25
#define RCC_APB1ENR_CAN2EN				26
#define RCC_APB1ENR_BKPEN				27
#define RCC_APB1ENR_PWREN				28
#define RCC_APB1ENR_DACEN				29

/*
 * Bit position definitions RCC_APB2ENR
 */
#define RCC_APB2ENR_AFIOEN				0
#define RCC_APB2ENR_IOPAEN				2
#define RCC_APB2ENR_IOPBEN				3
#define RCC_APB2ENR_IOPCEN				4
#define RCC_APB2ENR_IOPDEN				5
#define RCC_APB2ENR_IOPEEN				6
#define RCC_APB2ENR_ADC1EN				9
#define RCC_APB2ENR_ADC2EN				10
#define RCC_APB2ENR_TIM1EN				11
#define RCC_APB2ENR_SPI1EN				12
#define RCC_APB2ENR_USART1EN			14

/*
 * Bit position definitions RCC_APB1RSTR
 */
#define RCC_APB1RSTR_TIM2RST			0
#define RCC_APB1RSTR_TIM3RST			1
#define RCC_APB1RSTR_TIM4RST			2
#define RCC_APB1RSTR_TIM5RST			3
#define RCC_APB1RSTR_TIM6RST			4
#define RCC_APB1RSTR_TIM7RST			5
#define RCC_APB1RSTR_WWDGRST			11
#define RCC_APB1RSTR_SPI2RST			14
#define RCC_APB1RSTR_SPI3RST			15
#define RCC_APB1RSTR_USART2RST			17
#define RCC_APB1RSTR_USART3RST			18
#define RCC_APB1RSTR_UART4RST			19
#define RCC_APB1RSTR_UART5RST			20
#define RCC_APB1RSTR_I2C1RST			21
#define RCC_APB1RSTR_I2C2RST			22
#define RCC_APB1RSTR_CAN1RST			25
#define RCC_APB1RSTR_CAN2RST			26
#define RCC_APB1RSTR_BKPRST				27
#define RCC_APB1RSTR_PWRRST				28
#define RCC_APB1RSTR_DACRST				29

/*
 * Bit position definitions RCC_APB2RSTR
 */
#define RCC_APB2RSTR_AFIORST			0
#define RCC_APB2RSTR_IOPARST			2
#define RCC_APB2RSTR_IOPBRST			3
#define RCC_APB2RSTR_IOPCRST			4
#define RCC_APB2RSTR_IOPDRST			5
#define RCC_APB2RSTR_IOPERST			6
#define RCC_APB2RSTR_ADC1RST			9
#define RCC_APB2RSTR_ADC2RST			10
#define RCC_APB2RSTR_TIM1RST			11
#define RCC_APB2RSTR_SPI1RST			12
#define RCC_APB2RSTR_USART1RST			14

/*
 * Peripheral Base Address type casted to xxx_RegDef_t
 */
#define GPIOA 						((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 						((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 						((GPIO_RegDef_t*) GPIOG_BASEADDR)

#define AFIO						((AFIO_RegDef_t*) AFIO_BASEADDR)
#define RCC							((RCC_RegDef_t*)  RCC_BASEADDR)
#define EXTI						((EXTI_RegDef_t*) EXTI_BASEADDR)

#define SPI1						((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3						((SPI_RegDef_t*) SPI3_BASEADDR)

#define I2C1						((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2						((I2C_RegDef_t*) I2C2_BASEADDR)

#define USART1						((USART_RegDef_t*) USART1_BASEADDR)
#define USART2						((USART_RegDef_t*) USART2_BASEADDR)
#define USART3						((USART_RegDef_t*) USART3_BASEADDR)
#define UART4						((USART_RegDef_t*) UART4_BASEADDR)
#define UART5						((USART_RegDef_t*) UART5_BASEADDR)

/*
 * Clock Enable/Disable for Macros GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()				(RCC-> APB2ENR |= (1<<RCC_APB2ENR_IOPAEN))
#define GPIOB_PCLK_EN()				(RCC-> APB2ENR |= (1<<RCC_APB2ENR_IOPBEN))
#define GPIOC_PCLK_EN()				(RCC-> APB2ENR |= (1<<RCC_APB2ENR_IOPCEN))
#define GPIOD_PCLK_EN()				(RCC-> APB2ENR |= (1<<RCC_APB2ENR_IOPDEN))
#define GPIOE_PCLK_EN()				(RCC-> APB2ENR |= (1<<RCC_APB2ENR_IOPEEN))

#define GPIOA_PCLK_DI()				(RCC-> APB2ENR &= ~(1<<RCC_APB2ENR_IOPAEN))
#define GPIOB_PCLK_DI()				(RCC-> APB2ENR &= ~(1<<RCC_APB2ENR_IOPBEN))
#define GPIOC_PCLK_DI()				(RCC-> APB2ENR &= ~(1<<RCC_APB2ENR_IOPCEN))
#define GPIOD_PCLK_DI()				(RCC-> APB2ENR &= ~(1<<RCC_APB2ENR_IOPDEN))
#define GPIOE_PCLK_DI()				(RCC-> APB2ENR &= ~(1<<RCC_APB2ENR_IOPEEN))


/*
 * Clock Enable/Disable Macros for I2Cx Peripherals
 */
#define I2C1_PCLK_EN() 				(RCC-> APB1ENR |= (1<<RCC_APB1ENR_I2C1EN))
#define I2C2_PCLK_EN() 				(RCC-> APB1ENR |= (1<<RCC_APB1ENR_I2C2EN))

#define I2C1_PCLK_DI() 				(RCC-> APB1ENR &= ~(1<<RCC_APB1ENR_I2C1EN))
#define I2C2_PCLK_DI() 				(RCC-> APB1ENR &= ~(1<<RCC_APB1ENR_I2C2EN))


/*
 * Clock Enable/Disable Macros for SPIx Peripherals
 */
#define SPI1_PCLK_EN() 				(RCC-> APB2ENR |= (1<<RCC_APB2ENR_SPI1EN))
#define SPI2_PCLK_EN() 				(RCC-> APB1ENR |= (1<<RCC_APB1ENR_SPI2EN))
#define SPI3_PCLK_EN() 				(RCC-> APB1ENR |= (1<<RCC_APB1ENR_SPI3EN))

#define SPI1_PCLK_DI() 				(RCC-> APB2ENR &= ~ (1<<RCC_APB2ENR_SPI1EN))
#define SPI2_PCLK_DI() 				(RCC-> APB1ENR &= ~ (1<<RCC_APB1ENR_SPI2EN))
#define SPI3_PCLK_DI() 				(RCC-> APB1ENR &= ~ (1<<RCC_APB1ENR_SPI3EN))


/*
 * Clock Enable/Disable Macros for USARTx/UARTx Peripherals
 */
#define USART1_PCLK_EN()			(RCC-> APB2ENR |= (1<<RCC_APB2ENR_USART1EN))
#define USART2_PCLK_EN()			(RCC-> APB1ENR |= (1<<RCC_APB1ENR_USART2EN))
#define USART3_PCLK_EN()			(RCC-> APB1ENR |= (1<<RCC_APB1ENR_USART3EN))
#define UART4_PCLK_EN()				(RCC-> APB1ENR |= (1<<RCC_APB1ENR_UART4EN))
#define UART5_PCLK_EN()				(RCC-> APB1ENR |= (1<<RCC_APB1ENR_UART5EN))

#define USART1_PCLK_DI()			(RCC-> APB2ENR &= ~ (1<<RCC_APB2ENR_USART1EN))
#define USART2_PCLK_DI()			(RCC-> APB1ENR &= ~ (1<<RCC_APB1ENR_USART2EN))
#define USART3_PCLK_DI()			(RCC-> APB1ENR &= ~ (1<<RCC_APB1ENR_USART3EN))
#define UART4_PCLK_DI()				(RCC-> APB1ENR &= ~ (1<<RCC_APB1ENR_UART4EN))
#define UART5_PCLK_DI()				(RCC-> APB1ENR &= ~ (1<<RCC_APB1ENR_UART5EN))


/*
 * Alternate Function I/O Enable/Disable (AFIO)
 */
#define AFIO_PCLK_EN()				(RCC-> APB2ENR |= (1<<RCC_APB2ENR_AFIOEN))
#define AFIO_PCLK_DI()				(RCC-> APB2ENR &=~(1<<RCC_APB2ENR_AFIOEN))


/*
 * to Reset GPIOx AFIO SPI I2C Peripherals
 */
#define GPIOA_REG_RESET()			do{ (RCC-> APB2RSTR |= (1<<RCC_APB2RSTR_IOPARST));  (RCC-> APB2RSTR &= ~ (1<<RCC_APB2RSTR_IOPARST)); }while(0)
#define GPIOB_REG_RESET()			do{ (RCC-> APB2RSTR |= (1<<RCC_APB2RSTR_IOPBRST));	(RCC-> APB2RSTR &= ~ (1<<RCC_APB2RSTR_IOPBRST)); }while(0)
#define GPIOC_REG_RESET()			do{ (RCC-> APB2RSTR |= (1<<RCC_APB2RSTR_IOPCRST));	(RCC-> APB2RSTR &= ~ (1<<RCC_APB2RSTR_IOPCRST)); }while(0)
#define GPIOD_REG_RESET()			do{ (RCC-> APB2RSTR |= (1<<RCC_APB2RSTR_IOPDRST));	(RCC-> APB2RSTR &= ~ (1<<RCC_APB2RSTR_IOPDRST)); }while(0)
#define GPIOE_REG_RESET()			do{ (RCC-> APB2RSTR |= (1<<RCC_APB2RSTR_IOPERST));	(RCC-> APB2RSTR &= ~ (1<<RCC_APB2RSTR_IOPERST)); }while(0)
#define AFIO_REG_RESET()			do{ (RCC-> APB2RSTR |= (1<<RCC_APB2RSTR_AFIORST));	(RCC-> APB2RSTR &= ~ (1<<RCC_APB2RSTR_AFIORST)); }while(0)
#define SPI1_REG_RESET()			do{ (RCC-> APB2RSTR |= (1<<RCC_APB2RSTR_SPI1RST));  (RCC-> APB2RSTR &= ~ (1<<RCC_APB2RSTR_SPI1RST)); }while(0)
#define SPI2_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_SPI2RST));  (RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_SPI2RST)); }while(0)
#define SPI3_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_SPI3RST));  (RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_SPI3RST)); }while(0)
#define I2C1_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_I2C1RST));  (RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_I2C1RST)); }while(0)
#define I2C2_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_I2C2RST));  (RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_I2C2RST)); }while(0)
#define	USART1_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB2RSTR_USART1RST));(RCC-> APB1RSTR &= ~ (1<<RCC_APB2RSTR_USART1RST));}while(0)
#define USART2_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_USART2RST));(RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_USART2RST));}while(0)
#define USART3_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_USART3RST));(RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_USART3RST));}while(0)
#define UART4_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_UART4RST));(RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_UART4RST));  }while(0)
#define UART5_REG_RESET()			do{ (RCC-> APB1RSTR |= (1<<RCC_APB1RSTR_UART5RST));(RCC-> APB1RSTR &= ~ (1<<RCC_APB1RSTR_UART5RST));  }while(0)


/*
 * Returns port code for given GPIOx base address
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
								        (x == GPIOF) ? 5 :\
								        (x == GPIOG) ? 6 :0 )

/*
 * IRQ(İnterrupt Request) Number of STM32F1xx MCU
 */
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2         		36
#define IRQ_NO_SPI3         		51
#define IRQ_NO_I2C1_EV     			31
#define IRQ_NO_I2C1_ER    			32
#define IRQ_NO_I2C2_EV     			33
#define IRQ_NO_I2C2_ER    			34
#define IRQ_NO_USART1	   		 	37
#define IRQ_NO_USART2	   			38
#define IRQ_NO_USART3	    		39
#define IRQ_NO_UART4	    		52
#define IRQ_NO_UART5	    		53


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    			0
#define NVIC_IRQ_PRI15    			15

/*
 * Some Generic Definition
 */
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET
#define FLAG_SET					SET
#define FLAG_RESET					RESET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8


/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/

/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_PECERR					12
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15


/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
//#define USART_CR1_OVER8  				15


/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10


/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#include <gpio_driver.h>
#include <spi_driver.h>
#include <i2c_driver.h>
#include <usart_driver.h>
#include <rcc_driver.h>



#endif /* INC_STM32F103XX_H_ */
