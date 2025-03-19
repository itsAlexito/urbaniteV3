/**
 * @file stm32f4_button.h
 * @brief Header for stm32f4_button.c file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */
#ifndef STM32F4_BUTTON_H_
#define STM32F4_BUTTON_H_
/* Includes ------------------------------------------------------------------*/
/* Standard C includes */
#include <stdint.h>

/* HW dependent includes */
#include "stm32f4xx.h"
#include "stm32f4_system.h"

/* Defines and enums ----------------------------------------------------------*/
/* Defines */
 #define STM32F4_PARKING_BUTTON_GPIO GPIOC /*!< GPIO port of the user button */
 #define STM32F4_PARKING_BUTTON_PIN 13 /*!< GPIO pin of the user button */
 #define MODER13_AS_INPUT (STM32F4_GPIO_MODE_IN << STM32F4_PARKING_BUTTON_PIN * 2) /*<! Input mode for BUTTON_PIN in MODER register */


/* Function prototypes and explanation -------------------------------------------------*/
/**
 * @brief Auxiliary function to change the GPIO and pin of a button. This function is used for testing purposes mainly although it can be used in the final implementation if needed.
 *
 * @param button_id ID of the button to change.
 * @param p_port New GPIO port for the button.
 * @param pin New GPIO pin for the button.
 *
 */
void stm32f4_button_set_new_gpio(uint32_t button_id, GPIO_TypeDef *p_port, uint8_t pin);

#endif /* STM32F4_BUTTON_H_ */