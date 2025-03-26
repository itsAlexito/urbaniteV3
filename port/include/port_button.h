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

#define PORT_PARKING_BUTTON_ID 0 /*!< Button ID for the parking button */
#define PORT_PARKING_BUTTON_DEBOUNCE_TIME_MS 100 /*!< Debounce time for the parking button in milliseconds */



/* Function prototypes and explanation -------------------------------------------------*/
/**
 * @brief Configure the HW specifications of a given button
 * 
 * @param button_id 
 */
void port_button_init(uint32_t button_id);

/**
 * @brief Return the status of the button (pressed or not).
 * 
 * @param button_id 
 * @return true 
 * @return false 
 */
bool port_button_get_pressed(uint32_t button_id);

/**
 * @brief Get the value of the GPIO connected to the button.
 * 
 * @param button_id 
 * @return true 
 * @return false 
 */
bool port_button_get_value(uint32_t button_id);

/**
 * @brief Set the status of the button (pressed or not).
 * This function is used to force the status of the button. It is used to simulate the button press in the tests.
 * 
 * @param button_id 
 * @param pressed 
 */
void port_button_set_pressed(uint32_t button_id, bool pressed);

/**
 * @brief Get the status of the interrupt line connected to the button.
 * This function is used to check if the interrupt line of the button is pending. It is called from the ISR to check if the button has been pressed.
 * 
 * @param button_id 
 * @return true 
 * @return false 
 */
bool port_button_get_pending_interrupt(uint32_t button_id);

/**
 * @brief Clear the pending interrupt of the button.
 * This function is used to clear the pending interrupt of the button. It is called from the ISR to avoid unwanted interrupts.
 * 
 * @param button_id 
 */
void port_button_clear_pending_interrupt(uint32_t button_id);

/**
 * @brief Disable the interrupts of the button.
 * This function is used to disable the interrupts of the button. It is used in the unit tests to avoid unwanted interrupts.
 * 
 * @param button_id 
 */

void port_button_disable_interrupts(uint32_t button_id);
#endif