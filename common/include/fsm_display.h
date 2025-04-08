/**
 * @file fsm_display.h
 * @brief Header for fsm_display.c file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

#ifndef FSM_DISPLAY_SYSTEM_H_
#define FSM_DISPLAY_SYSTEM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "fsm.h"
/* Standard C includes */

/* Defines and enums ----------------------------------------------------------*/
/* Enums */
enum FSM_DISPLAY_SYSTEM
{
    WAIT_DISPLAY = 0,
    SET_DISPLAY
};



/* Defines and enums ----------------------------------------------------------*/
#define DANGER_MIN_CM  0//PREGUNTAR
#define WARNING_MIN_CM  25
#define NO_PROBLEM_MIN_CM  50
#define INFO_MIN_CM  150
#define OK_MIN_CM  175
#define OK_MAX_CM  200

/* Typedefs --------------------------------------------------------------------*/
typedef struct fsm_display_t fsm_display_t; /*!< Variable */

/* Function prototypes and explanation -------------------------------------------------*/

/**
 * @brief Create
 * 
 * @param display_id 
 * @return fsm_display_t* 
 */
fsm_display_t *fsm_display_new(uint32_t display_id);

/**
 * @brief Destroy a FSM display
 * this function destroys the FSM display and frees the memory allocated for it.
 * 
 * @param p_fsm 
 */
void fsm_display_destroy(fsm_display_t *p_fsm);

/**
 * @brief set the display system to show the distance
 * this function is used to set the display system to show the distance. The distance is set in cm.
 * 
 * @param p_fsm 
 * @param distance_cm 
 */
void fsm_display_set_distance(fsm_display_t *p_fsm, uint32_t distance_cm);

/**
 * @brief Fire the FSM display
 * This function is used to fire the FSM. It is used to check  the transitions and execute the actions of the FSM.
 * 
 * @param p_fsm 
 */
void fsm_display_fire(fsm_display_t *p_fsm);

/**
 * @brief get the status of the display FSM
 * This function returns the status of the display system.
 * This function might be used for testing and debugging purposes.
 * 
 * @param p_fsm 
 * @return true  If the display system has been indicated to be active.
 * @return false If the display system has been indicated to be paused
 */
bool fsm_display_get_status(fsm_display_t *p_fsm);

/**
 * @brief Set the status of the display FSM.
 * This function is used to set the status of the display system.
 * Indicating if the display system is active or paused.
 * 
 * @param p_fsm 
 * @param pause Status of the display system. true if the display system is paused, false if the display system is active.
 */
void fsm_display_set_status(fsm_display_t *p_fsm, bool pause);

/**
 * @brief 
 * 
 * @param p_fsm 
 * @return true 
 * @return false 
 */
bool fsm_display_check_activity(fsm_display_t *p_fsm);

/**
 * @brief  get the inner FSM of the display system
 * this function returns the inner FSM of the display system
 * 
 * @param p_fsm 
 * @return fsm_t* 
 */
fsm_t *fsm_display_get_inner_fsm(fsm_display_t *p_fsm);

/**
 * @brief 
 * 
 * @param p_fsm 
 * @return uint32_t 
 */
uint32_t fsm_display_get_state(fsm_display_t *p_fsm);

/**
 * @brief 
 * 
 * @param p_fsm 
 * @param state 
 */
void fsm_display_set_state(fsm_display_t *p_fsm, int8_t state);

/**
 * @brief 
 * 
 * @param p_fsm 
 * @return true 
 * @return false 
 */
bool fsm_display_check_activity(fsm_display_t *p_fsm);

#endif /* FSM_DISPLAY_SYSTEM_H_ */