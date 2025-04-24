/**
 * @file fsm_ultrasound.h
 * @brief Header for fsm_ultrasound.c file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

#ifndef FSM_ULTRASOUND_H_
#define FSM_ULTRASOUND_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "fsm.h"
/* Standard C includes */

/* Defines and enums ----------------------------------------------------------*/
#define FSM_ULTRASOUND_NUM_MEASUREMENTS 5 /*!< Number of measurements to be taken */

/* Typedefs --------------------------------------------------------------------*/
typedef struct fsm_ultrasound_t fsm_ultrasound_t; /*!< Variable */
/* Function prototypes and explanation -------------------------------------------------*/

/* Enums */

enum FSM_ULTRASOUND
{
    WAIT_START = 0, 
    TRIGGER_START,
    WAIT_ECHO_START,
    WAIT_ECHO_END,
    SET_DISTANCE,
};

/**
 * @brief Set the state of the ultrasound FSM.
 *
 * This function sets the current state of the ultrasound FSM.
 *
 * > &nbsp;&nbsp;&nbsp;&nbsp;💡 This function is important because the struct is private and external functions such as those of the unit tests cannot access the state of the FSM directly. \n
 * 
 * @param p_fsm Pointer to an `fsm_ultrasound_t` struct.
 * @param state New state of the ultrasound FSM.
 */
void fsm_ultrasound_set_state(fsm_ultrasound_t *p_fsm, int8_t state);

/**
 * @brief 
 * 
 * @param ultrasound_id 
 * @return fsm_ultrasound_t* 
 */
fsm_ultrasound_t *fsm_ultrasound_new(uint32_t ultrasound_id);

/**
 * @brief Destroy an ultrasound FSM.
 * This function destroys an ultrasound transceiver FSM and frees the memory.
 * 
 * @param p_fsm 
 */
void fsm_ultrasound_destroy(fsm_ultrasound_t *p_fsm);

/**
 * @brief Return the distance of the last object detected by the ultrasound sensor.a64l.
 * The function also resets the field new_measurement to indicate that the distance has been read.
 * 
 * @param p_fsm 
 * @return uint32_t 
 */
uint32_t fsm_ultrasound_get_distance(fsm_ultrasound_t *p_fsm);

/**
 * @brief Fire the ultrasound FSM.
 * This function is used to fire the ultrasound FSM.
 * It is used to check the transitions and execute the actions of the ultrasound FSM.
 * 
 * @param p_fsm 
 */
void fsm_ultrasound_fire(fsm_ultrasound_t *p_fsm);

/**
 * @brief Get the status of the ultrasound transceiver FSM.
 * This function returns the status of the ultrasound. This function might be used for testing and debugging purposes.
 * 
 * @param p_fsm 
 * @param status 
 * @return true if the ultrasound is active
 * @return false if not
 */
bool fsm_ultrasound_get_status(fsm_ultrasound_t *p_fsm);

/**
 * @brief Set the status of the ultrasound sensor. 
 * true means that the ultrasound sensor is active and a distance measurement must be performed.
 * false means that the ultrasound sensor is inactive.
 * 
 * @param p_fsm 
 * @param status 
 */
void fsm_ultrasound_set_status(fsm_ultrasound_t *p_fsm, bool status);

/**
 * @brief Get the ready flag of the trigger signal in the ultrasound HW.
 * This function returns the ready flag of trigger signal in the ultrasound HW.
 * This function might be used for testing and debugging purposes.
 * 
 * @param p_fsm 
 * @return true If the port indicates that the trigger signal is ready to start a new measurement.
 * @return false false If the port indicates that the trigger signal is not ready to start a new measurement.
 */
bool fsm_ultrasound_get_ready(fsm_ultrasound_t *p_fsm);

/**
 * @brief Return the flag that indicates if a new measurement is ready.
 * 
 * @param p_fsm 
 * @return true 
 * @return false 
 */
bool fsm_ultrasound_get_new_measurement_ready(fsm_ultrasound_t *p_fsm);

/**
 * @brief Stop the ultrasound sensor.
 * This function stops the ultrasound sensor by indicating to the port to stop the ultrasound sensor (to reset all timer ticks)
 * and to set the status of the ultrasound sensor to inactive.
 * 
 * @param p_fsm 
 */
void fsm_ultrasound_stop(fsm_ultrasound_t *p_fsm);

/**
 * @brief Start the ultrasound sensor.
 * his function starts the ultrasound sensor by indicating to the port to start 
 * the ultrasound sensor (to reset all timer ticks) and to set the status of the ultrasound sensor to active.
 * 
 * @param p_fsm 
 */
void fsm_ultrasound_start(fsm_ultrasound_t *p_fsm);

/**
 * @brief Get the inner FSM of the ultrasound.
 * This function returns the inner FSM of the ultrasound
 * 💡 This function is important because the struct is private and external functions such as those of the unit tests cannot access the inner FSM directly.
 * 
 * @param p_fsm 
 * @return fsm_t* 
 */
fsm_t *fsm_ultrasound_get_inner_fsm(fsm_ultrasound_t *p_fsm);

/**
 * @brief Get the state of the ultrasound FSM.
 * This function returns the current state of the ultrasound FSM.
 * 💡 This function is important because the struct is private and external functions such as those of the unit tests cannot access the state of the FSM directly.
 * 
 * @param p_fsm 
 * @return uint32_t 
 */
uint32_t fsm_ultrasound_get_state(fsm_ultrasound_t *p_fsm);

/**
 * @brief Set the state of the ultrasound FSM
 * This function sets the current state of the ultrasound FSM.
 * 💡 This function is important because the struct is private and external functions such as those of the unit tests cannot access the state of the FSM directly.
 * 
 * @param p_fsm 
 * @param state 
 */
void fsm_ultrasound_set_state(fsm_ultrasound_t *p_fsm, int8_t state);

/**
 * @brief Check if the ultrasound sensor is doing a distance measurement.
 * The ultrasound sensor is always inactive because all the transitions are due to HW interrupts.
 * 
 * @param p_fsm 
 * @return true 
 * @return false 
 */
bool fsm_ultrasound_check_activity(fsm_ultrasound_t *p_fsm);



#endif /* FSM_ULTRASOUND_H_ */
