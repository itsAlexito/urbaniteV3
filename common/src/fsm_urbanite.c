/**
 * @file fsm_display.c
 * @brief Display system FSM main file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */
/* Includes ------------------------------------------------------------------*/
/* Standard C includes */
#include <stdlib.h>
#include <stdio.h>
#include "port_system.h"
#include "fsm.h"
#include "fsm_urbanite.h"


/* Struct */
struct fsm_urbanite_t
{
    fsm_t f; /*!< Urbanite FSM */
    fsm_button_t *p_fsm_button; /*!< Pointer to the button FSM */
    uint32_t on_off_press_time_ms; /*!< Time in ms to consider ON/OFF of the Urbanite parking aid system */
    uint32_t pause_display_time_ms; /*!< Time in ms to pause the display system */
    bool is_paused; /*!< Flag indicating if the display system is paused */
    fsm_ultrasound_t *p_fsm_ultrasound_rear; /*!< Pointer to the rear ultrasound FSM */
    fsm_display_t *p_fsm_display_rear; /*!< Pointer to the rear display FSM */
};

/* Private functions -----------------------------------------------------------*/
// /* State machine input or transition functions */

/**
 * @brief Check if the button has been pressed for the required time to turn ON the Urbanite system.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_on(fsm_t *p_this)
{
    //✅ 1. Call function fsm_button_get_duration() to get the duration of the button press
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    uint32_t duration = fsm_button_get_duration(p_fsm_urbanite->p_fsm_button);
    //✅ 2. Return true if the duration is greater than 0 and greater than the required time to turn ON the system. Otherwise, return false
    return (duration > 0 && duration > p_fsm_urbanite->on_off_press_time_ms);
}

/**
 * @brief Check if the button has been pressed for the required time to turn OFF the system.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_off(fsm_t *p_this)
{
    //✅ 1. It is enough to call the function check_on() because the required time to turn ON and OFF the system is the same.
    return check_on(p_this);
}

/**
 * @brief Check if a new measurement is ready.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_new_measure(fsm_t *p_this)
{
    //✅ 1. Call function fsm_ultrasound_get_new_measurement_ready() to check if a new measurement is ready and return the result.
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    return fsm_ultrasound_get_new_measurement_ready(p_fsm_urbanite->p_fsm_ultrasound_rear);
}

/**
 * @brief Check if it has been required to pause the display.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_pause_display(fsm_t *p_this)
{
    //✅ 1. Call function fsm_button_get_duration() to get the duration of the button press
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    uint32_t duration = fsm_button_get_duration(p_fsm_urbanite->p_fsm_button);
    //✅ 2. Return true if the duration is greater than 0, less than the required time to turn ON the system, and greater than the required time to pause the display. Otherwise, return false
    return (duration > 0 && duration < p_fsm_urbanite->on_off_press_time_ms && duration > p_fsm_urbanite->pause_display_time_ms);
}

/**
 * @brief Check if any of the elements of the system is active.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_activity(fsm_t *p_this)
{
    //✅ 1. Return true if any of the elements (button, ultrasound, or display) is active. Otherwise, return false
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    return (fsm_button_check_activity(p_fsm_urbanite->p_fsm_button) ||
            fsm_ultrasound_get_status(p_fsm_urbanite->p_fsm_ultrasound_rear) ||
            fsm_display_get_status(p_fsm_urbanite->p_fsm_display_rear));
}

/**
 * @brief Check if all the elements of the system are inactive.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_no_activity(fsm_t *p_this)
{
    //✅ 1. Call function check_activity() and return the inverse of the result. The result will be true if all the elements of the system are inactive, otherwise it will be false.
    return !check_activity(p_this);
}

/**
 * @brief Check if any a new measurement is ready while the system is in low power mode.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_activity_in_measure(fsm_t *p_this)
{
  //✅ 1. Call function check_new_measure() to check if a new measurement is ready and return the result.
    return check_new_measure(p_this);
}

/* --------------------------------- State machine output or action functions -------------------------------*/

/**
 * @brief Turn the Urbanite system ON.
 * 
 * @param p_this 
 */
static void do_start_up_measure(fsm_t *p_this)
{
    //✅ 1. Reset the duration of the button by calling fsm_button_reset_duration() to avoid the system to turn OFF again.
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    fsm_button_reset_duration(p_fsm_urbanite->p_fsm_button);
    //✅ 2. Start the ultrasound sensor by calling its appropriate function with the right parameter. With this the ultrasound sensor will start measuring the distance.
    fsm_ultrasound_start(p_fsm_urbanite->p_fsm_ultrasound_rear);
    //✅ 3. Set the appropriate status of the display system by calling its function with the right parameter. With this the display system will start showing the distance.
    fsm_display_set_status(p_fsm_urbanite->p_fsm_display_rear, true);
    //    💡 Print a message to help yourself to debug and log the status of the system. You could do something like:
    printf("[URBANITE][%ld] Urbanite system ON\n", port_system_get_millis());
}


static void do_stop_urbanite(fsm_t *p_this)
{
    //✅ 1. Reset the duration of the button by calling fsm_button_reset_duration() to avoid the system to turn ON again.
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    fsm_button_reset_duration(p_fsm_urbanite->p_fsm_button);
    //✅ 2. Stop the ultrasound sensor by calling fsm_ultrasound_stop() with the right parameters. With this the ultrasound sensor will stop measuring the distance if it was measuring.
    fsm_ultrasound_stop(p_fsm_urbanite->p_fsm_ultrasound_rear);
    //✅ 3. Turn the display system off by calling fsm_display_set_status() with the right parameter. With this the display system will stop showing the distance if it was showing it
    fsm_display_set_status(p_fsm_urbanite->p_fsm_display_rear, false);
    //✅ 4. If the system is paused, remove the pause status to avoid the system to turn ON again with the display paused.
    if (p_fsm_urbanite->is_paused)
    {
        fsm_display_set_status(p_fsm_urbanite->p_fsm_display_rear, false);
        p_fsm_urbanite->is_paused = false;
    }
    //✅ 5. Print a message to help yourself to debug and log the status of the system. You could do something like:
    printf("[URBANITE][%ld] Urbanite system OFF\n", port_system_get_millis());
}

static void do_pause_display(fsm_t* p_this)
{
    //✅ 1. Reset the duration of the button by calling fsm_button_reset_duration() to avoid the system to pause again.
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    fsm_button_reset_duration(p_fsm_urbanite->p_fsm_button);
    //✅ 2. Invert the pause status. If the system is paused, set the pause status to false. If the system is not paused, set the pause status to true.
    p_fsm_urbanite->is_paused = !p_fsm_urbanite->is_paused;
    //✅ 3. Activate or deactivate the display depending on the new pause status by calling fsm_display_set_status() with the right parameter.
    fsm_display_set_status(p_fsm_urbanite->p_fsm_display_rear, p_fsm_urbanite->is_paused);
    //✅ 4. Print status depending on the pause status. You could print messages like:
    if (p_fsm_urbanite->is_paused)
    {
        printf("[URBANITE][%ld] Display paused\n", port_system_get_millis());
    }
    else
    {
        printf("[URBANITE][%ld] Display resumed\n", port_system_get_millis());
    }
}

/**
 * @brief Display the distance measured by the ultrasound sensor.
 * 
 * @param p_this 
 */
static void do_display_distance(fsm_t* p_this)
{
    //✅ 1. Get the distance measured by the ultrasound sensor by calling fsm_ultrasound_get_distance() with the right ultrasound sensor.
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t *)p_this;
    uint32_t distance_cm = fsm_ultrasound_get_distance(p_fsm_urbanite->p_fsm_ultrasound_rear);
    //2. If the system is paused:      If the distance is less than WARNING_MIN_CM / 2 cm, set the distance to the display and set the display status to true. Otherwise, set the display status to false.
    //If the system is not paused, set the distance to the display
    if (p_fsm_urbanite->is_paused)
    {
        if (distance_cm < WARNING_MIN_CM / 2)
        {
            fsm_display_set_distance(p_fsm_urbanite->p_fsm_display_rear, distance_cm);
            fsm_display_set_status(p_fsm_urbanite->p_fsm_display_rear, true);
        }
        else
        {
            fsm_display_set_status(p_fsm_urbanite->p_fsm_display_rear, false);
        }
    }
    else
    {
        fsm_display_set_distance(p_fsm_urbanite->p_fsm_display_rear, distance_cm);
    }
    //✅ 3. Print a message to help yourself to debug and log the distance measured by the ultrasound sensor. You could do something like:
    printf("[URBANITE][%ld] Distance: %ld cm\n", port_system_get_millis(), distance_cm);
}

/**
 * @brief Start the low power mode while the Urbanite is OFF.
 * 
 * @param p_this 
 */
static void do_sleep_off(fsm_t *p_this)
{
    //✅ 1. Call function port_system_sleep() to start the low power mode
    port_system_sleep();
}

/**
 * @brief Start the low power mode while the Urbanite is measuring the distance and it is waiting for a new measurement.
 * 
 * @param p_this 
 */
static void do_sleep_while_measure(fsm_t *p_this)
{
    //✅ 1. Call function port_system_sleep() to start the low power mode
    port_system_sleep();
}

/**
 * @brief Start the low power mode while the Urbanite is awakened by a debug breakpoint or similar in the SLEEP_WHILE_ON state.
 * 
 * @param p_this 
 */
static void do_sleep_while_off(fsm_t *p_this)
{
    //✅ 1. Call function port_system_sleep() to start the low power mode
    port_system_sleep();
}

/**
 * @brief Start the low power mode while the Urbanite is awakened by a debug breakpoint or similar in the SLEEP_WHILE_ON state.
 * 
 * @param p_this 
 */
static void do_sleep_while_on(fsm_t *p_this)
{
    //✅ 1. Call function port_system_sleep() to start the low power mode
    port_system_sleep();
}

static fsm_trans_t fsm_trans_urbanite[] = {
    { OFF, check_on, MEASURE, do_start_up_measure }, //
    { OFF, check_no_activity, SLEEP_WHILE_OFF, do_sleep_off }, //
    { SLEEP_WHILE_OFF, check_activity, OFF, NULL }, //
    { SLEEP_WHILE_OFF, check_no_activity, SLEEP_WHILE_OFF, do_sleep_while_off },//
    { MEASURE, check_off, OFF, do_stop_urbanite }, //
    { MEASURE, check_pause_display, MEASURE, do_pause_display }, //
    { MEASURE, check_new_measure, MEASURE, do_display_distance }, //
    { MEASURE, check_no_activity, SLEEP_WHILE_ON, do_sleep_while_measure }, //
    { SLEEP_WHILE_ON, check_activity_in_measure, MEASURE, NULL }, //
    { SLEEP_WHILE_ON, check_no_activity, SLEEP_WHILE_ON, do_sleep_while_on },
    { -1, NULL, -1, NULL },
};

/* Other auxiliary functions */

/**
 * @brief Create a new Urbanite FSM.
 * 
 * @param p_fsm_urbanite 
 * @param p_fsm_button 
 * @param on_off_press_time_ms 
 * @param pause_display_time_ms 
 * @param p_fsm_ultrasound_rear 
 * @param p_fsm_display_rear 
 */
static void fsm_urbanite_init(fsm_urbanite_t *p_fsm_urbanite,
                              fsm_button_t *p_fsm_button,
                              uint32_t on_off_press_time_ms,
                              uint32_t pause_display_time_ms,
                              fsm_ultrasound_t *p_fsm_ultrasound_rear,
                              fsm_display_t *p_fsm_display_rear)
{
    //✅ 1. Call function fsm_init() with the received pointer to fsm_t and the transition table.
    fsm_init(&p_fsm_urbanite->f, fsm_trans_urbanite);
    //✅ 2. Initialize the fields p_fsm_button, on_off_press_time_ms p_fsm_ultrasound_rear, pause_display_time_ms, p_fsm_display_rear and p_fsm_display_rear of the Urbanite FSM with the received parameters.
    p_fsm_urbanite->p_fsm_button = p_fsm_button;
    p_fsm_urbanite->on_off_press_time_ms = on_off_press_time_ms;
    p_fsm_urbanite->pause_display_time_ms = pause_display_time_ms;
    p_fsm_urbanite->p_fsm_ultrasound_rear = p_fsm_ultrasound_rear;
    p_fsm_urbanite->p_fsm_display_rear = p_fsm_display_rear;
    //✅ 3. Initialize the field is_paused to false.
    p_fsm_urbanite->is_paused = false;

}

/* Public functions -----------------------------------------------------------*/


fsm_urbanite_t *fsm_urbanite_new(fsm_button_t *p_fsm_button,
                                 uint32_t on_off_press_time_ms,
                                 uint32_t pause_display_time_ms,
                                 fsm_ultrasound_t *p_fsm_ultrasound_rear,
                                 fsm_display_t *p_fsm_display_rear)
{
    //✅ 1. Allocate memory for the fsm_urbanite_t struct in the same way as the fsm_button_new(), fsm_ultrasound_new() and fsm_display_new() functions.
    fsm_urbanite_t *p_fsm_urbanite = (fsm_urbanite_t*)malloc(sizeof(fsm_urbanite_t));
    //✅ 2. Initialize the fsm_t struct of the Urbanite FSM by calling the fsm_urbanite_init() function.
    fsm_urbanite_init(p_fsm_urbanite, p_fsm_button, on_off_press_time_ms, pause_display_time_ms, p_fsm_ultrasound_rear, p_fsm_display_rear);
    //✅ 3. Return the pointer to the Urbanite FSM.
    return p_fsm_urbanite;
}

void fsm_urbanite_fire(fsm_urbanite_t *p_fsm)
{
    // Call the fire function of the FSM
    fsm_fire(&p_fsm->f);
}

void fsm_urbanite_destroy(fsm_urbanite_t *p_fsm)
{
    free(p_fsm); // Free the memory of the FSM
}