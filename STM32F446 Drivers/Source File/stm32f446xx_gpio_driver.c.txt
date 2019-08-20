/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: 21-Jul-2019
 *      Author: chaya kumar gowda(Kumar A Chaya)
 */


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx.h"



/*
 * peripheral clock setup
 */
/********************************************************************************************
 * @fn                            -  GPIO_periPheral Clock
 *
 * @breif                         -  This enables or disables the peripheral clock of GPIO pin
 *
 * @param[in]                     -  Base address of GPIO peripheral
 * @param[in]                     -  ENABLE or DISABLE macros
 * @param[in]                     -
 *
 * @return                        -  none
 *
 * @Note                          -  none
 */


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else if(EnorDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
			{
				GPIOA_PCLK_DI();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_PCLK_DI();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_PCLK_DI();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_PCLK_DI();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_PCLK_DI();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_PCLK_DI();
			}

	}
}



/*
 * gpio initialize and deinitialize
 */

/********************************************************************************************
 * @fn                            -  GPIO Init
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */


void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	uint32_t temp = 0;

	//ENABLE THE PERPHERAL CLOCK BEFORE INITIALIZATION
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//configure GPIO pins
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//configure falling edge trigger
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//configure for raising edge trigger
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//configure for raising edge and falling edge trigger
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//enable the EXTI interrupt delivered using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		//configure the GPIO port selection in SYSCFG_EXTICR
        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;  //THERE ARE 4 EXCTI REGISTERS IN SYSCFG PERIPHERAL HENCE, IT IS / BY 4
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;  // FOR BETTER UNDERSTANDING REFER EXTI REGISTERS
        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();   //enabling system clock before configuration
        SYSCFG->EXTICR[temp1] |= portcode << (temp2 * 4);
 	}
	temp = 0;

	//configure the speed

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//configure the pull up pull down settings

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	//configure the output type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < GPIO_PIN_NO_8)
		{
			pGPIOHandle->pGPIOx->AFRL &= ~( 0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}
		else
		{
			pGPIOHandle->pGPIOx->AFRH &= ~( 0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}
	}


}

/********************************************************************************************
 * @fn                            -     GPIO De Init
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx == GPIOA)
			{
				GPIOA_REG_RESET();
			}
			else if(pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if(pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
}


/*
 * data read and write
 */

/********************************************************************************************
 * @fn                            -    GPIO data read from the input pin
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -    0 or 1
 *
 * @Note                          -
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;

}

/********************************************************************************************
 * @fn                            -     GPIO data read from the input port
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;

	return value;

}

/********************************************************************************************
 * @fn                            -		GPIO data write to the output pin
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to the output data register bit field corresponding to the pin number
		pGPIOx->ODR |=(1 << PinNumber);
	}
	else
	{
		//write 0 to the output data register bit field corresponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/********************************************************************************************
 * @fn                            -		GPIO data write to the output port
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/********************************************************************************************
 * @fn                            -  GPIO Toggle Output 
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^=(1 << PinNumber);

}

/*
 * IRQ configuration and ISR handling
 */
/********************************************************************************************
 * @fn                            -   GPIO IRQ Interrupt
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program for ISER0 register
			//*NVIC is dereferencing
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program for ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >=64 && IRQNumber < 96)
		{
			//program for ISER2 register
			*NVIC_ISER1 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program for ICER0 register
			//*NVIC is dereferencing
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{

			//program for ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);

		}else if(IRQNumber >=64 && IRQNumber < 96)
		{

			//program for ICER2 register
			*NVIC_ICER1 |= (1 << IRQNumber % 64);

		}

	}

}

/********************************************************************************************
 * @fn                            -  GPIO IRQ Priority
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

void GPIO_IRQPriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber/4;  //there are registers of IPR each IPR has 4 sections. So divide by 4
	uint8_t iprx_section = IRQNumber % 4; // to select which section of respective IPR

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTATION); // Because last 4 bits in every priority register is unused
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << (8 * shift_amount));

}

/********************************************************************************************
 * @fn                            -   GPIO IRQHandling
 *
 * @breif                         -
 *
 * @param[in]                     -
 * @param[in]                     -
 * @param[in]                     -
 *
 * @return                        -
 *
 * @Note                          -
 */

void GPIO_IRQHandling(uint8_t PinNumber)   //IRQ handling should from which pin the interrupt is triggered
{
	if(EXTI->PR & (1 << PinNumber)) // if the PR corresponding to the pin number is set,
	{
		// clearing PR is by setting 1
		EXTI->PR |= (1 << PinNumber);
	}

}
