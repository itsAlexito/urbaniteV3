/**
 * @file fsm_urbanite.h
 * @brief Header for fsm_urbanite.c file.
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
#include "fsm_buzzer.h"  // NUEVO: para integrar FSM del buzzer

/* Typedefs --------------------------------------------------------------------*/
typedef struct fsm_urbanite_t fsm_urbanite_t; /*!< Variable */

/* Enums */
 /**
  * @brief Enumerator for the Urbanite finite state machine.
  * 
  */
enum FSM_URBANITE {
    OFF = 0,
    MEASURE,
    SLEEP_WHILE_OFF,
    SLEEP_WHILE_ON
};

/**
 * @brief Destroy the urbanite FSM.
 */
void fsm_urbanite_destroy(fsm_urbanite_t *p_fsm);

/**
 * @brief Fire the Urbanite FSM.
 */
void fsm_urbanite_fire(fsm_urbanite_t *p_fsm);

/**
 * @brief Create a new Urbanite FSM.
 * 
 * @param p_fsm_button FSM del botón
 * @param on_off_press_time_ms Tiempo para activar/desactivar sistema
 * @param pause_display_time_ms Tiempo para pausar el display
 * @param p_fsm_ultrasound_rear FSM del sensor trasero
 * @param p_fsm_display_rear FSM del display trasero
 * @param p_fsm_buzzer_rear FSM del buzzer trasero
 * @return fsm_urbanite_t* 
 */
fsm_urbanite_t *fsm_urbanite_new(fsm_button_t* p_fsm_button,
                                 uint32_t on_off_press_time_ms,
                                 uint32_t pause_display_time_ms,
                                 fsm_ultrasound_t* p_fsm_ultrasound_rear,
                                 fsm_display_t* p_fsm_display_rear,
                                 fsm_buzzer_t* p_fsm_buzzer_rear);  // NUEVO

#endif /* FSM_URBANITE_H_ */
