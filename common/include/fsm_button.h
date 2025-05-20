/**
 * @file fsm_button.h
 * @brief Header for fsm_button.c file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

 #ifndef FSM_BUTTON_H_
 #define FSM_BUTTON_H_ /*!< Variable */
 
 /* Includes ------------------------------------------------------------------*/
 /* Standard C includes */
 #include <stdint.h>
 #include <stdbool.h>
 
 /* Other includes */
 #include "fsm.h"
 
 /* Defines and enums ----------------------------------------------------------*/
 /* Enums */

 /**
  * @brief Enum for the button FSM
  * 
  */
 enum FSM_BUTTON
 {
     BUTTON_RELEASED = 0,
     BUTTON_RELEASED_WAIT,
     BUTTON_PRESSED,
     BUTTON_PRESSED_WAIT,
 };
 
 
 /* Typedefs --------------------------------------------------------------------*/
 typedef struct fsm_button_t fsm_button_t; /*!< Variable */
 
 
 /* Function prototypes and explanation -------------------------------------------------*/
 
 /**
 * @brief Create a new button FSM. Reserve memory for the FSM and initialize it.
  * 
  * @param debounce_time_ms 
  * @param button_id 
  * @return fsm_button_t* 
  */
 fsm_button_t *fsm_button_new(uint32_t debounce_time_ms, uint32_t button_id);


 /**
 * @brief Destroy the FSM. Free the memory reserved for the FSM.
 * 
 * @param p_fsm 
 */
 void fsm_button_destroy(fsm_button_t *p_fsm);
 
/**
 * @brief Fire the FSM. Call the fsm_fire function with the FSM pointer.
 * 
 * @param p_fsm 
 */
 void fsm_button_fire(fsm_button_t *p_fsm);
 
 /**
 * @brief Get the inner FSM of the button.
 * 
 * @param p_fsm 
 * @return fsm_t* 
 */
 fsm_t *fsm_button_get_inner_fsm(fsm_button_t *p_fsm);
 
 /**
 * @brief Get the state of the button FSM.
 * 
 * @param p_fsm 
 * @return uint32_t 
 */
 uint32_t fsm_button_get_state(fsm_button_t *p_fsm);
 
 
/**
 * @brief Get the duration of the button pressed.
 * 
 * @param p_fsm 
 * @return uint32_t 
 */
 uint32_t fsm_button_get_duration(fsm_button_t *p_fsm);
 
 
 /**
  * @brief Reset the duration of the button pressed to 0
  * 
  * @param p_fsm 
  */
 void fsm_button_reset_duration(fsm_button_t *p_fsm);
 
 
 /**
  * @brief 
  * 
  * @param p_fsm 
  * @return uint32_t 
  */
 uint32_t fsm_button_get_debounce_time_ms(fsm_button_t *p_fsm);
 
 
 /**
  * @brief Check if the button FSM is active, or not.
  * The button is inactive when it is in the status BUTTON_RELEASED.

  * 
  * @param p_fsm 
  * @return true 
  * @return false 
  */
 bool fsm_button_check_activity(fsm_button_t *p_fsm);
 
 
 
 
 
 
 #endif