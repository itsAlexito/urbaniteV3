/**
 * @file fsm_ultrasound.h
 * @brief Header for fsm_ultrasound.c file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

 #ifndef FSM_URBANITE_H_
 #define FSM_URBANITE_H_

 /* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include "fsm_button.h"
#include "fsm_display.h"
#include "fsm_ultrasound.h"


/* Typedefs --------------------------------------------------------------------*/
typedef struct fsm_urbanite_t fsm_urbanite_t; /*!< Variable */

/* Enums */
enum FSM_URBANITE {
    OFF = 0, /*!< Starting state. Also comes here when the button has been pressed for the required time to turn off the urbanite */   
    MEASURE, /*!< State to measure the distance to the obstacles */
    SLEEP_WHILE_OFF, /*!< State to start the low power mode while the urbanite is off*/
    SLEEP_WHILE_ON /*!< State to start the low power mode while the urbanite is on*/
};


/**
 * @brief Destroy the urbanite FSM.
 * This function destroys an Urbanite FSM and frees the memory.
 * 
 * @param p_fsm 
 */
void fsm_urbanite_destroy(fsm_urbanite_t *p_fsm);

/**
 * @brief Fire the Urbanite FSM.
 * This function is used to check the transitions and execute the actions of the Urbanite FSM.
 * 
 * 
 * @param p_fsm 
 */
void fsm_urbanite_fire(fsm_urbanite_t *p_fsm);

/**
 * @brief Create a new Urbanite FSM.
 * This function creates a new Urbanite FSM with the given button, ultrasound, display FSMs and the required times for configuration.
 * 
 * 
 * @param p_fsm_button  Pointer to the button FSM to interact with the Urbanite.
 * @param on_off_press_time_ms Time in ms to consider ON/OFF of the Urbanite parking aid system. 
 * @param pause_display_time_ms Time in ms to pause the display system.
 * @param p_fsm_ultrasound_rear Pointer to the rear ultrasound FSM.
 * @param p_fsm_display_rear Pointer to the rear display FSM.
 * @return fsm_urbanite_t* Pointer to the Urbanite FSM.
 */
fsm_urbanite_t *fsm_urbanite_new(fsm_button_t* p_fsm_button,
                                 uint32_t on_off_press_time_ms,
                                 uint32_t pause_display_time_ms,
                                 fsm_ultrasound_t* p_fsm_ultrasound_rear,
                                 fsm_display_t* p_fsm_display_rear);


#endif /* FSM_URBANITE_H_ */    