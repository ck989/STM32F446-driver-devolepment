/*
 * stm32f446xx_gpio_driver.h
 *
 *  Created on: 21-Jul-2019
 *      Author: Kumar A Chaya (chaya kumar gowda)
 */

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/*
 * this is a configuration for gpio pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;            //@GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;              //@GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;             //@GPIO_PIN_SPEED
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;



/*
 * this is a structure handle for gpio pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;            //this holds the address of the gpio pin which it belongs to.
	GPIO_PinConfig_t GPIO_PinConfig;  //this holds gpio configuration settings.
}GPIO_Handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */

#define GPIO_PIN_NO_0               0
#define GPIO_PIN_NO_1               1
#define GPIO_PIN_NO_2               2
#define GPIO_PIN_NO_3               3
#define GPIO_PIN_NO_4               4
#define GPIO_PIN_NO_5               5
#define GPIO_PIN_NO_6               6
#define GPIO_PIN_NO_7               7
#define GPIO_PIN_NO_8               8
#define GPIO_PIN_NO_9               9
#define GPIO_PIN_NO_10              10
#define GPIO_PIN_NO_11              11
#define GPIO_PIN_NO_12              12
#define GPIO_PIN_NO_13              13
#define GPIO_PIN_NO_14              14
#define GPIO_PIN_NO_15              15


/*
 * @GPIO_PIN_MODES
 * GPIO possible modes
 */

#define GPIO_MODE_IN                  0
#define GPIO_MODE_OUT                 1
#define GPIO_MODE_ALTFN               2
#define GPIO_MODE_ANALOG              3
#define GPIO_MODE_IT_FT               4  //falling edge trigger
#define GPIO_MODE_IT_RT               5  //raising edge trigger
#define GPIO_MODE_IT_RFT              6  //raising falling edge trigger


/*
 * GPIO output type register
 */

#define GPIO_OP_TYPE_PP               0
#define GPIO_OP_TYPE_OD               1

/*
 * @GPIO_PIN_SPEED
 * GPIO possible speed modes
 */

#define GPIO_SPEED_LOW                0
#define GPIO_SPEED_MEDIUM             1
#define GPIO_SPEED_HIGH               2
#define GPIO_SPEED_VERY_HIGH          3

/*
 * GPIO pull up and pull down configuration macros
 */

#define GPIO_NO_PUPD                  0
#define GPIO_PIN_PU                   1
#define GPIO_PIN_PD                   2

/*
 *
 */
/****************************************************************************************************************
 *                                        API's supported by the drivers
 *****************************************************************************************************************/

/*
 * gpio initialize and deinitialize
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * peripheral clock setup
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and ISR handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);                                  //IRQ handling should from which pin the interrupt s














#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
