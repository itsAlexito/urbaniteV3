/**
 * @file port_button.h
 * @brief Header for the portable functions to interact with the HW of the buttons. The functions must be implemented in the platform-specific code.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

#ifndef PORT_BUTTON_H_
#define PORT_BUTTON_H_

/* Includes ------------------------------------------------------------------*/
/* Standard C includes */
#include <stdint.h>
#include <stdbool.h>

/* Defines and enums ----------------------------------------------------------*/
/* Defines */
// Define here all the button identifiers that are used in the system

#define PORT_PARKING_BUTTON_ID 0
#define PORT_PARKING_BUTTON_DEBOUNCE_TIME_MS 100



/* Function prototypes and explanation -------------------------------------------------*/
/**
 * @brief 
 * 
 * @param button_id 
 */
void port_button_init(uint32_t button_id);

/**
 * @brief 
 * 
 * @param button_id 
 * @return true 
 * @return false 
 */
bool port_button_get_pressed(uint32_t button_id);

/**
 * @brief 
 * 
 * @param button_id 
 * @return true 
 * @return false 
 */
bool port_button_get_value(uint32_t button_id);

/**
 * @brief 
 * 
 * @param button_id 
 * @param pressed 
 */

/**
 * @brief 
 * 
 * @param button_id 
 * @param pressed 
 */
void port_button_set_pressed(uint32_t button_id, bool pressed);

/**
 * @brief 
 * 
 * @param button_id 
 * @return true 
 * @return false 
 */
bool port_button_get_pending_interrupt(uint32_t button_id);

/**
 * @brief 
 * 
 * @param button_id 
 */
void port_button_clear_pending_interrupt(uint32_t button_id);

/**
 * @brief 
 * 
 * @param button_id 
 */

void port_button_disable_interrupts(uint32_t button_id);
#endif