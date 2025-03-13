/**
 * @file stm32f4_ultrasound.h
 * @brief Header for stm32f4_ultrasound.c file.
 * @author alumno1
 * @author alumno2
 * @date date
 */
#ifndef STM32F4_ULTRASOUND_H_
#define STM32F4_ULTRASOUND_H_

/* Includes ------------------------------------------------------------------*/
/* Standard C includes */
#include <stdint.h>
#include "stm32f4xx.h"

/* HW dependent includes */

/* Defines and enums ----------------------------------------------------------*/
/* Defines */

/**
 * @brief  TRIGGER(ENVIA){GPIO:  , PIN: PB0} , ECHO(RECIBE){GPIO:  , PIN: PA1}
 * 
 */

#define STM32F4_REAR_PARKING_SENSOR_TRIGGER_GPIO GPIOB
#define STM32F4_REAR_PARKING_SENSOR_TRIGGER_PIN 0
#define STM32F4_REAR_PARKING_SENSOR_ECHO_GPIO GPIOA
#define STM32F4_REAR_PARKING_SENSOR_ECHO_PIN 1 


//-------------
#define MODER13_AS_OUTPUT (STM32F4_GPIO_MODE_OUT << STM32F4_REAR_PARKING_SENSOR_TRIGGER_PIN  * 2)       /*<! Output mode for BUTTON_PIN in MODER register */
#define MODER13_AS_ALTERNATE (STM32F4_GPIO_MODE_AF << STM32F4_REAR_PARKING_SENSOR_ECHO_PIN  * 2)       /*<! Alternate mode for BUTTON_PIN in MODER register */

/* Function prototypes and explanation -------------------------------------------------*/
/**
 * @brief Auxiliary function to change the GPIO and pin of the trigger pin of an ultrasound transceiver. This function is used for testing purposes mainly although it can be used in the final implementation if needed.
 *
 * @param ultrasound_id ID of the trigger signal to change.
 * @param p_port New GPIO port for the trigger signal.
 * @param pin New GPIO pin for the trigger signal.
 *
 */
void stm32f4_ultrasound_set_new_trigger_gpio(uint32_t ultrasound_id, GPIO_TypeDef *p_port, uint8_t pin);

/**
 * @brief Auxiliary function to change the GPIO and pin of the echo pin of an ultrasound transceiver. This function is used for testing purposes mainly although it can be used in the final implementation if needed.
 *
 * @param ultrasound_id ID of the echo signal to change.
 * @param p_port New GPIO port for the echo signal.
 * @param pin New GPIO pin for the echo signal.
 *
 */
void stm32f4_ultrasound_set_new_echo_gpio(uint32_t ultrasound_id, GPIO_TypeDef *p_port, uint8_t pin);


#endif /* STM32F4_ULTRASOUND_H_ */
