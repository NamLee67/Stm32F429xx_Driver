/*
 * stm32f429xx_gpio_driver.h
 *
 *  Created on: Aug 7, 2024
 *      Author: ACER
 */

#ifndef INC_STM32F429XX_GPIO_DRIVER_H_
#define INC_STM32F429XX_GPIO_DRIVER_H_

#include "stm32f429xx.h"
#define __vo volatile

typedef struct{
	__vo uint32_t MODER;		// GPIO port mode register						offset 0x00
	__vo uint32_t OTYPER;		// GPIO port output type register				offset 0x04
	__vo uint32_t OSPEEDR;		// GPIO port output speed register				offset 0x08
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register			offset 0x0C
	__vo uint32_t IDR;			// GPIO port input data register 				offset 0x10
	__vo uint32_t ODR;			// GPIO port output data register				offset 0x14
	__vo uint32_t BSRR;			// GPIO port bit set/reset register				offset 0x18
	__vo uint32_t LCKR;			// GPIO port configuration lock register		offset 0x1C
	__vo uint32_t AFR[2];		// GPIO alternate function obtain low and high register			offset 0x20 to 0x24
}GPIO_RegDef_t;

/*
 * This is a configuration structure for a GPIO pin
 */
typedef struct
{
	uint8_t GPIO_PinNumber;			/* possiible values from @GPIO_pin_numbers*/
	uint8_t GPIO_PinMode;			/* possiible values from @GPIO_pin_mode*/
	uint8_t GPIO_PinSpeed;			/* possiible values from @GPIO_pin_output_speeds*/
	uint8_t GPIO_PinPuPdControl;	/* possiible values from @GPIO_pin_pull_up_and_pull_down_configuration*/
	uint8_t GPIO_PinOPType;			/* possiible values from @GPIO_pin_output_types*/
	uint8_t GPIO_PinAltFunMode;		/* possiible values from @GPIO_pin_mode*/
}GPIO_PinConfig_t;

/*
 * this is a Handle Structure for a GPIO pin
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx;				// pointer to base address GPIOx port
	GPIO_PinConfig_t GPIO_PinConfig;	// GPIO pin will configration in this variable
}GPIO_Handle_t;


/*
 * @GPIO_pin_numbers
 */
#define GPIO_PIN_NO_0			0
#define GPIO_PIN_NO_1			1
#define GPIO_PIN_NO_2			2
#define GPIO_PIN_NO_3			3
#define GPIO_PIN_NO_4			4
#define GPIO_PIN_NO_5			5
#define GPIO_PIN_NO_6			6
#define GPIO_PIN_NO_7			7
#define GPIO_PIN_NO_8			8
#define GPIO_PIN_NO_9			9
#define GPIO_PIN_NO_10			10
#define GPIO_PIN_NO_11			11
#define GPIO_PIN_NO_12			12
#define GPIO_PIN_NO_13			13
#define GPIO_PIN_NO_14			14
#define GPIO_PIN_NO_15			15


/*
 * @GPIO_pin_mode
 */
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * @GPIO_pin_output_types
 */
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

/*
 * @GPIO_pin_output_speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_pin_pull_up_and_pull_down_configuration
 */
#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2


/***********************************************************************************
 * 							APIs supported by this driver
 * 			For more information about the APIs check the function definitions
 ***********************************************************************************
 */

/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Read and Write data
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ config and handler
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);








#endif /* INC_STM32F429XX_GPIO_DRIVER_H_ */
