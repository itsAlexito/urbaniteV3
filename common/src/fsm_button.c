/**
 * @file fsm_button.c
 * @brief Button FSM main file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

/* Includes ------------------------------------------------------------------*/
/* Standard C includes */

/* HW dependent includes */
#include "port_button.h"
#include "port_system.h"
#include <stdlib.h>

/* Project includes */
#include "fsm.h"
#include "fsm_button.h"

/* Structure fsm_button_t*/

struct fsm_button_t
{
    fsm_t f; /* Base struct for the FSM */
    uint32_t debounce_time_ms; /* Debounce time in milliseconds */
    uint32_t next_timeout; /* Next time out */
    uint32_t tick_pressed; /* Tick pressed */
    uint32_t duration; /* Duration */
    uint32_t button_id; /* Button ID */
};

/* State machine input or transition functions */   //INPUTTTTTT!!!!
/**
 * @brief Check if the button has been pressed, Call function port_button_get_pressed and return the value 
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_button_pressed(fsm_t *p_this)
{
    return port_button_get_pressed(((fsm_button_t *)p_this)->button_id);
}

/**
 * @brief Check if the button has been released, Call function port_button_get_pressed and return the opposite value
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_button_released(fsm_t *p_this)
{
    return !port_button_get_pressed(((fsm_button_t *)p_this)->button_id);
}


/**
 * @brief Check if the debounce time has passed, Call function port_system_get_millis and compare it with the next_timeout value
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_timeout(fsm_t *p_this)
{
    return port_system_get_millis() >= ((fsm_button_t *)p_this)->next_timeout;
}


/* State machine output or action functions */ //OUTPUTTTTT!!!!

/**
 * @brief Store the system tick when the button was pressed. Call function port_system_get_millis and store the value in tick_pressed, also set the next_timeout value
 * 
 * @param p_this 
 */
static void do_store_tick_pressed(fsm_t *p_this)
{
    ((fsm_button_t *)p_this)->tick_pressed = port_system_get_millis();
    ((fsm_button_t *)p_this)->next_timeout = port_system_get_millis() + ((fsm_button_t *)p_this)->debounce_time_ms;
}

/**
 * @brief Store the duration of the button pressed. Call function port_system_get_millis and store the value in duration, also set the next_timeout value
 * 
 * @param p_this 
 */
static void do_set_duration(fsm_t *p_this)
{
    ((fsm_button_t *)p_this)->duration = port_system_get_millis() - ((fsm_button_t *)p_this)->tick_pressed;
    ((fsm_button_t *)p_this)->next_timeout = port_system_get_millis() + ((fsm_button_t *)p_this)->debounce_time_ms;
}

static fsm_trans_t fsm_trans_button[] = {
    {BUTTON_RELEASED, check_button_pressed, BUTTON_PRESSED_WAIT, do_store_tick_pressed},
    {BUTTON_PRESSED_WAIT, check_timeout, BUTTON_PRESSED, NULL},
    {BUTTON_PRESSED, check_button_released, BUTTON_RELEASED_WAIT, do_set_duration},
    {BUTTON_RELEASED_WAIT, check_timeout, BUTTON_RELEASED, NULL},
    {-1, NULL, -1, NULL}

};


/* Other auxiliary functions */
static void fsm_button_init(fsm_button_t *p_fsm_button, uint32_t debounce_time, uint32_t button_id)
{
    fsm_init(&p_fsm_button->f, fsm_trans_button);

    /* TODO alumnos: */

    //Initialize the fields of the button with the received parameters
    p_fsm_button->debounce_time_ms = debounce_time;
    p_fsm_button->button_id = button_id;
    p_fsm_button->tick_pressed = 0;
    fsm_button_reset_duration(p_fsm_button); //Initialize the duration to 0
    port_button_init(button_id); //Initialize the button
    

}

/* Public functions -----------------------------------------------------------*/
fsm_button_t *fsm_button_new(uint32_t debounce_time, uint32_t button_id)
{
    fsm_button_t *p_fsm_button = (fsm_button_t *)malloc(sizeof(fsm_button_t));  /* Do malloc to reserve memory of all other FSM elements, although it is interpreted as fsm_t (the first element of the structure) */
    fsm_button_init(p_fsm_button, debounce_time, button_id);   /* Initialize the FSM */
    return p_fsm_button;                                       /* Composite pattern: return the fsm_t pointer as a fsm_button_t pointer */
}

uint32_t fsm_button_get_duration(fsm_button_t *p_fsm)
{
    return p_fsm->duration;
}

void fsm_button_reset_duration(fsm_button_t *p_fsm)
{
    p_fsm->duration = 0;
}


uint32_t fsm_button_get_debounce_time_ms(fsm_button_t *p_fsm)
{
    return p_fsm->debounce_time_ms;
}
/* FSM-interface functions. These functions are used to interact with the FSM */
void fsm_button_fire(fsm_button_t *p_fsm)
{
    fsm_fire(&p_fsm->f); // Is it also possible to it in this way: fsm_fire((fsm_t *)p_fsm);
}

void fsm_button_destroy(fsm_button_t *p_fsm)
{
    free(&p_fsm->f);
}

fsm_t *fsm_button_get_inner_fsm(fsm_button_t *p_fsm)
{
    return &p_fsm->f;
}

uint32_t fsm_button_get_state(fsm_button_t *p_fsm)
{
    return p_fsm->f.current_state;
}