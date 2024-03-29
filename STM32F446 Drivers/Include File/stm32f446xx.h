/*
 * stm32f446xx.h
 *
 *  Created on: 21-Jul-2019
 *      Author: chaya kumar gowda
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_


#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak  __attribute__((weak))

/**********************************************PROCESSOR SPECIFIC DETAILS****************************************************/
/*
 * ARM CORTEX Mx processor NVIC ISERx register address
 */

#define NVIC_ISER0                          (__vo uint32_t*)0xE000E100
#define NVIC_ISER1                          (__vo uint32_t*)0xE000E104
#define NVIC_ISER2                          (__vo uint32_t*)0xE000E108
#define NVIC_ISER3                          (__vo uint32_t*)0xE000E10C

/*
 * ARM CORTEX Mx processor NVIC ICERx register address
 */


#define NVIC_ICER0                          (__vo uint32_t*)0xE000E180
#define NVIC_ICER1                          (__vo uint32_t*)0xE000E184
#define NVIC_ICER2                          (__vo uint32_t*)0xE000E188
#define NVIC_ICER3                          (__vo uint32_t*)0xE000E18C



/*
 * NVIC priority base address
 */

#define NVIC_PR_BASE_ADDR                  (__vo uint32_t*)0xE000E400

#define NO_PR_BITS_IMPLEMENTATION           4


//base addresses of flash and sram memories

#define FLASH_BASEADDR                      0x08000000U   //fetched from datasheet
#define SRAM1_BASEADDR                      0x20000000U   //112KB
#define SRAM2_BASEADDR                      0x2001C000U   //size of sram1 + base address of sram
#define ROM_BASEADDR                        0x1FFF0000U   //fetched from datasheet
#define SRAM                                SRAM1_BASEADDR

// base addresses of peripheral bus

#define PERIPH_BASEADDR                     0x40000000U
#define APB1_PERIPH_BASEADDR                PERIPH_BASEADDR
#define APB2_PERIPH_BASEADDR                0x40010000U
#define AHB1_PERIPH_BASEADDR                0x40020000U
#define AHB2_PERIPH_BASEADDR                0x50000000U

//base addresses of peripherals which are connected to AHB1 buses

#define GPIOA_BASEADDR                      (AHB1_PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR                      (AHB1_PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR                      (AHB1_PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR                      (AHB1_PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR                      (AHB1_PERIPH_BASEADDR + 0x1000)
#define GPIOH_BASEADDR                      (AHB1_PERIPH_BASEADDR + 0x1C00)
#define RCC_BASEADDR                        (AHB1_PERIPH_BASEADDR + 0x3800)

//base addresses of peripherals which are connected APB1 buses

#define I2C1_BASEADDR                       (APB1_PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR                       (APB1_PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR                       (APB1_PERIPH_BASEADDR + 0x5C00)

#define SPI2_BASEADDR                       (APB1_PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR                       (APB1_PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR                     (APB1_PERIPH_BASEADDR + 0x4400)

//base addresses of peripherals which are connected to APB2 buses

#define EXTI_BASEADDR                       (APB2_PERIPH_BASEADDR + 0x3C00)

#define SYSCFG_BASEADDR                     (APB2_PERIPH_BASEADDR + 0x3800)

#define SPI1_BASEADDR                       (APB2_PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR                       (APB2_PERIPH_BASEADDR + 0x3400)

#define USART1_BASEADDR                     (APB2_PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR                     (APB2_PERIPH_BASEADDR + 0x1400)


/*****************************************peripheral register definition structures******************************/


typedef struct
{
	__vo uint32_t MODER;                        //GPIO port mode register               offset address 0x00
	__vo uint32_t OTYPER;                       //GPIO port output type register        offset address 0x40
	__vo uint32_t OSPEEDR;                      //GPIO port output speed register       offset address 0x80
	__vo uint32_t PUPDR;                        //GPIO port pull-up/pull-down register  offset address 0xC0
	__vo uint32_t IDR;                          //GPIO port input data register         offset address 0x10
	__vo uint32_t ODR;                          //GPIO port output data register        offset address 0x14
	__vo uint32_t BSRR;                         //GPIO port bit set/reset register      offset address 0x18
	__vo uint32_t LCKR;                         //GPIO port configuration lock register offset address 0x1C
	__vo uint32_t AFRL;                         //GPIO alternate function low register  offset address 0x20
	__vo uint32_t AFRH;	                       //GPIO alternate function high register  offset address 0x24
}GPIO_RegDef_t;


typedef struct
{
	__vo uint32_t CR;                                  //Address offset 0x00
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	 uint32_t RESERVED0;
	 uint32_t RESERVED1;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	 uint32_t RESERVED2;
	 uint32_t RESERVED3;
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	 uint32_t RESERVED4;
	 uint32_t RESERVED5;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	 uint32_t RESERVED6;
	 uint32_t RESERVED7;
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	 uint32_t RESERVED8;
 	 uint32_t RESERVED9;
 	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	 uint32_t RESERVED10;
	 uint32_t RESERVED11;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	 uint32_t RESERVED12;
	 uint32_t RESERVED13;
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t DCKCFGR;                                       //Address offset 0x8C
}RCC_RegDef_t;

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;      //pending register

}EXTI_RegDef_t;

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];

}SYSCFG_RegDef_t;

typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;

}SPI_RegDef_t;

typedef struct
{

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;

}I2C_RegDef_t;

typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;

}USART_RegDef_t;
//peripheral definition typecasted to xxx_RefDef_t

#define GPIOA                         ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                         ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                         ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                         ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                         ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH                         ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC                           ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI                          ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG                        ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI2                          ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3                          ((SPI_RegDef_t*)SPI3_BASEADDR)

#define SPI1                          ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI4                          ((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1                          ((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2                          ((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3                          ((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1                        ((USART_RegDef_t*)USART1_BASEADDR)
#define USART6                        ((USART_RegDef_t*)USART6_BASEADDR)
#define USART2                        ((USART_RegDef_t*)USART2_BASEADDR)

/*
 * clock enable Macros peripherals for GPIO
 */

#define GPIOA_PCLK_EN()                ( RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()                ( RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()                ( RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()                ( RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()                ( RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()                ( RCC->AHB1ENR |= (1 << 7))

/*
 * clock enable Macros peripherals for I2C
 */

#define I2C1_PCLK_EN()                 ( RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()                 ( RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()                 ( RCC->APB1ENR |= (1 << 23))

/*
 * clock enable Macros peripherals for SPI
 */

#define SPI1_PCLK_EN()                 ( RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()                 ( RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()                 ( RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()                 ( RCC->APB2ENR |= (1 << 13))

/*
 * clock enable Macros peripherals for SPI
 */

#define USART1_PCLK_EN()               ( RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()               ( RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()               ( RCC->APB2ENR |= (1 << 5))

/*
 * clock enable Macros peripherals for SYSCFG
 */

#define SYSCFG_PCLK_EN()               ( RCC->APB2ENR |= (1 << 14))

/*
 * clock disable Macros peripherals for GPIO
 */

#define GPIOA_PCLK_DI()                ( RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()                ( RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()                ( RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()                ( RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()                ( RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()                ( RCC->AHB1ENR &= ~(1 << 7))

/*
 * clock disable Macros peripherals for I2C
 */

#define I2C1_PCLK_DI()                 ( RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()                 ( RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()                 ( RCC->APB1ENR &= ~(1 << 23))

/*
 * clock disable Macros peripherals for SPI
 */

#define SPI1_PCLK_DI()                 ( RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()                 ( RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()                 ( RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()                 ( RCC->APB2ENR &= ~(1 << 13))

/*
 * clock disable Macros peripherals for SPI
 */

#define USART1_PCLK_DI()               ( RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()               ( RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()               ( RCC->APB2ENR &= ~(1 << 5))

/*
 * clock disable Macros peripherals for SYSCFG
 */

#define SYSCFG_PCLK_DI()               ( RCC->APB2ENR &= ~(1 << 14))

/*
 * GPIO registers RESET
 */

#define GPIOA_REG_RESET()              do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()              do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()              do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()              do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()              do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOH_REG_RESET()              do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)

/*
 * SPI register RESET
 */

#define SPI1_REG_RESET()              do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()              do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()              do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)
#define SPI4_REG_RESET()              do{ (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); }while(0)

/*
 * I2C REGISTER RESET
 */

#define I2C1_REG_RESET()              do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()              do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()              do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

/*
 * USART REGISTER RESET
 */

#define USART1_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()           do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART6_REG_RESET()           do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)



/*
 * returns port code for the respective GPIOx base address
 */

#define GPIO_BASEADDR_TO_CODE(x)       ( (x == GPIOA) ? 0 : \
		                                 (x == GPIOB) ? 1 : \
		                                 (x == GPIOC) ? 2 : \
		                                 (x == GPIOD) ? 3 : \
		                                 (x == GPIOE) ? 4 : \
		                                 (x == GPIOH) ? 7 : 0 )

/*
 * IRQ(Interrupt Request) Number of STM32F446RE
 */

#define IRQ_NO_EXTI0                   6
#define IRQ_NO_EXTI1                   7
#define IRQ_NO_EXTI2                   8
#define IRQ_NO_EXTI3                   9
#define IRQ_NO_EXTI4                   10
#define IRQ_NO_EXTI9_5                 23
#define IRQ_NO_EXTI15_10               40
#define IRQ_NO_SPI1                    35
#define IRQ_NO_SPI2                    36
#define IRQ_NO_SPI3                    51
#define IRQ_NO_I2C1_EV                 31
#define IRQ_NO_I2C1_ERR                32
#define IRQ_NO_I2C2_EV                 33
#define IRQ_NO_I2C2_ERR                34
#define IRQ_NO_I2C3_EV                 79
#define IRQ_NO_I2C3_ERR                80

/*
 * MACROS for all priority levels
 */

#define NVIC_IRQ_PRI0                  0
#define NVIC_IRQ_PRI1                  1
#define NVIC_IRQ_PRI2                  2
#define NVIC_IRQ_PRI3                  3
#define NVIC_IRQ_PRI4                  4
#define NVIC_IRQ_PRI5                  5
#define NVIC_IRQ_PRI6                  6
#define NVIC_IRQ_PRI7                  7
#define NVIC_IRQ_PRI8                  8
#define NVIC_IRQ_PRI9                  9
#define NVIC_IRQ_PRI10                 10
#define NVIC_IRQ_PRI11                 11
#define NVIC_IRQ_PRI12                 12
#define NVIC_IRQ_PRI13                 13
#define NVIC_IRQ_PRI14                 14
#define NVIC_IRQ_PRI15                 15



/*
 * some genric macros
 */

#define ENABLE                            1
#define DISABLE                           0
#define SET                               ENABLE
#define RESET                             DISABLE
#define GPIO_PIN_SET                      SET
#define GPIO_PIN_RESET                    RESET
#define FLAG_SET                          SET
#define FLAG_RESET                        RESET



/****************************************************************************************************************
 *                                 BIT POSITION DEFINATION FOR SPI PERIPHERALS
 ****************************************************************************************************************/

/*
 * BIT POSITION DEFINITION FOR CR1
 */

#define SPI_CR1_CPHA                      0
#define SPI_CR1_CPOL                      1
#define SPI_CR1_MSTR                      2
#define SPI_CR1_BR                        3
#define SPI_CR1_SPE                       6
#define SPI_CR1_LSBFIRST                  7
#define SPI_CR1_SSI                       8
#define SPI_CR1_SSM                       9
#define SPI_CR1_RXONLY                    10
#define SPI_CR1_DFF                       11
#define SPI_CR1_CRCNEXT                   12
#define SPI_CR1_CRCEN                     13
#define SPI_CR1_BIDIOE                    14
#define SPI_CR1_BIDIMODE                  15


/*
 * BIT POSITION DEFINITION FOR CR2
 */

#define SPI_CR2_RXDMAEN                    0
#define SPI_CR2_TXDMAEN                    1
#define SPI_CR2_SSOE                       2
#define SPI_CR2_FRF                        4
#define SPI_CR2_ERRIE                      5
#define SPI_CR2_RXNEIE                     6
#define SPI_CR2_TXEIE                      7

/*
 * BIT POSITION DEFINITION FOR SR
 */

#define SPI_SR_RXNE                        0
#define SPI_SR_TXE                         1
#define SPI_SR_CHSIDE                      2
#define SPI_SR_UDR                         3
#define SPI_SR_CRCERR                      4
#define SPI_SR_MODF                        5
#define SPI_SR_OVR                         6
#define SPI_SR_BSY                         7
#define SPI_SR_FRE                         8




/****************************************************************************************************************
 *                                 BIT POSITION DEFINATION FOR I2C PERIPHERALS
 ****************************************************************************************************************/

/*
 * BIT POSITION DEFINITION FOR CR1
 */

#define I2C_CR1_PE                         0
#define I2C_CR1_SMBUS                      1
#define I2C_CR1_SMBTYPE                    3
#define I2C_CR1_ENARP                      4
#define I2C_CR1_ENPEC                      5
#define I2C_CR1_ENGC                       6
#define I2C_CR1_NOSTRETCH                  7
#define I2C_CR1_START                      8
#define I2C_CR1_STOP                       9
#define I2C_CR1_ACK                        10
#define I2C_CR1_POS                        11
#define I2C_CR1_PEC                        12
#define I2C_CR1_ALERT                      13
#define I2C_CR1_SWRST                      14

/*
 * BIT POSITION DEFINITION FOR CR2
 */

#define I2C_CR2_FREQ                       0
#define I2C_CR2_ITERREN                    8
#define I2C_CR2_ITEVTEN                    9
#define I2C_CR2_ITBUFEN                    10
#define I2C_CR2_DMAEN                      11
#define I2C_CR2_LAST                       12

/*
 * BIT POSITION DEFINITION FOR OAR1
 */

#define I2C_OAR1_ADD0                      0
#define I2C_OAR1_ADD17                     1
#define I2C_OAR1_ADD89                     8
#define I2C_OAR1_ADDMODE                   15

/*
 * BIT POSITION DEFINITION FOR SR1
 */

#define I2C_SR1_SB                         0
#define I2C_SR1_ADDR                       1
#define I2C_SR1_BTF                        2
#define I2C_SR1_ADD10                      3
#define I2C_SR1_STOPF                      4
#define I2C_SR1_RXNE                       6
#define I2C_SR1_TXE                        7
#define I2C_SR1_BERR                       8
#define I2C_SR1_ARLO                       9
#define I2C_SR1_AF                         10
#define I2C_SR1_OVR                        11
#define I2C_SR1_PECER                      12
#define I2C_SR1_TIMEOUT                    13
#define I2C_SR1_SMBALERT                   14

/*
 * BIT POSITION DEFINITION FOR SR2
 */

#define I2C_SR2_MSL                        0
#define I2C_SR2_BUSY                       1
#define I2C_SR2_TRA                        2
#define I2C_SR2_GENCALL                    4
#define I2C_SR2_SMBDEFAULT                 5
#define I2C_SR2_SMBHOST                    6
#define I2C_SR2_DUAL                       7

/*
 * BIT POSITION DEFINITION FOR CCR
 */

#define I2C_CCR_CCR                        0
#define I2C_CCR_DUTY                       1
#define I2C_CCR_FS                         2

/****************************************************************************************************************
 *                                 BIT POSITION DEFINATION FOR USART PERIPHERALS
 ****************************************************************************************************************/


/*
 * BIT POSITION DEFINITION FOR SR
 */

#define USART_SR_PE                         0
#define USART_SR_FE                         1
#define USART_SR_NF                         2
#define USART_SR_ORE                        3
#define USART_SR_IDLE                       4
#define USART_SR_RXNE                       5
#define USART_SR_TC                         6
#define USART_SR_TXE                        7
#define USART_SR_LBD                        8
#define USART_SR_CTS                        9

/*
 * BIT POSITION DEFINITION FOR DR
 */

#define USART_DR_DR08                       0

/*
 * BIT POSITION DEFINITION FOR BRR
 */

#define USART_BRR_DIV_Fraction03            0
#define USART_BRR_DIV_Mantissa011           4

/*
 * BIT POSITION DEFINITION FOR CR1
 */

#define USART_CR1_SBK                       0
#define USART_CR1_RWU                       1
#define USART_CR1_RE                        2
#define USART_CR1_TE                        3
#define USART_CR1_IDLEIE                    4
#define USART_CR1_RXNEIE                    5
#define USART_CR1_TCIE                      6
#define USART_CR1_TXEIE                     7
#define USART_CR1_PEIE                      8
#define USART_CR1_PS                        9
#define USART_CR1_PCE                       10
#define USART_CR1_WAKE                      11
#define USART_CR1_M                         12
#define USART_CR1_UE                        13
#define USART_CR1_OVER8                     15

/*
 * BIT POSITION DEFINITION FOR CR2
 */

#define USART_CR2_ADD                       0
#define USART_CR2_LBDL                      5
#define USART_CR2_LBDIE                     6
#define USART_CR2_LBCL                      8
#define USART_CR2_CPHA                      9
#define USART_CR2_CPOL                      10
#define USART_CR2_CLKEN                     11
#define USART_CR2_STOP                      12
#define USART_CR2_LINEN                     14

/*
 * BIT POSITION DEFINITION FOR CR3
 */

#define USART_CR3_EIE                       0
#define USART_CR3_IREN                      1
#define USART_CR3_IRLP                      2
#define USART_CR3_HDSEL                     3
#define USART_CR3_NACK                      4
#define USART_CR3_SCEN                      5
#define USART_CR3_DMAR                      6
#define USART_CR3_DMAT                      7
#define USART_CR3_RTSE                      8
#define USART_CR3_CTSE                      9
#define USART_CR3_CTSIE                     10
#define USART_CR3_ONEBIT                    11


/*
 * BIT POSITION DEFINITION FOR GTPR
 */

#define USART_GTPR_PSC                     0
#define USART_GTPR_GT                      8



#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"










#endif /* INC_STM32F446XX_H_ */
