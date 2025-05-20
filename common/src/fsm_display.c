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


/* HW dependent includes */

/* Project includes */
#include "port_display.h"
#include "port_system.h"
#include "fsm_display.h"
#include "fsm.h"
/* Typedefs --------------------------------------------------------------------*/
/**
 * @brief Structure of the Display FSM.
 * 
 */
struct fsm_display_t
{
    fsm_t f; /*!< Display FSM */
    uint32_t distance_cm; /*!< Distance measured by the display sensor */
    bool new_color; /*!< Flag indicating if a new color is available */
    bool status; /*!< Status of the display */
    bool idle ; /*!< Flag indicating if the display is idle */
    uint32_t display_id; /*!< ID of the display, must be unique */
};

/* Private functions -----------------------------------------------------------*/

/**
 * @brief 
 * 
 * @param p_color 
 * @param distance_cm 
 */
void _compute_display_levels(rgb_color_t *p_color, int32_t distance_cm)
{
    if (p_color == NULL) return;

    if (distance_cm >= DANGER_MIN_CM && distance_cm <= WARNING_MIN_CM) // rojo
    {
        *p_color = COLOR_RED; 
    }
    else if (distance_cm > WARNING_MIN_CM && distance_cm <= NO_PROBLEM_MIN_CM) // amarillo
    {
        *p_color = COLOR_YELLOW; 
    }
    else if (distance_cm > NO_PROBLEM_MIN_CM && distance_cm <= INFO_MIN_CM) // verde
    {
        *p_color = COLOR_GREEN; 
    }
    else if (distance_cm > INFO_MIN_CM && distance_cm <= OK_MIN_CM) // turquesa
    {
        *p_color = COLOR_TURQUOISE; 
    }
    else if (distance_cm > OK_MIN_CM && distance_cm <= OK_MAX_CM) // azul
    {
        *p_color = COLOR_BLUE; 
    }
    else
    {
        *p_color = COLOR_OFF; // no color
    }
}

/* State machine input or transition functions */

/**
 * @brief check if the display is set to be active (on), independently if it is idle or not.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_active(fsm_t *p_this)
{
    fsm_display_t *p_fsm_display = (fsm_display_t *)p_this;
    return p_fsm_display->status;
}

/**
 * @brief check if a new color has to be set
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_set_new_color(fsm_t *p_this)
{
    fsm_display_t *p_fsm_display = (fsm_display_t *)p_this;
    return p_fsm_display->new_color;
}

/**
 * @brief check if the display is set to be inactive (off)
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_off(fsm_t *p_this)
{
    fsm_display_t *p_fsm_display = (fsm_display_t *)p_this;
    return !p_fsm_display->status;
}

/* State machine output or action functions */

/**
 * @brief turn the display sistem ON for the first time
 * 
 * @param p_this 
 */
static void do_set_on(fsm_t *p_this)
{
    //starts ON with no color 
    fsm_display_t *p_fsm_display = (fsm_display_t *)p_this;
    port_display_set_rgb(p_fsm_display->display_id, COLOR_OFF);
}

/**
 * @brief set the color of the RGB LEDs according to the distance
 * 
 * @param p_this 
 */
static void do_set_color(fsm_t *p_this)
{
    //✅ 1. Compute the levels of the RGB LEDs according to the distance and set the display level
    fsm_display_t *p_fsm_display = (fsm_display_t *)p_this;
    rgb_color_t color;
    _compute_display_levels(&color, p_fsm_display->distance_cm);
    port_display_set_rgb(p_fsm_display->display_id, color);
    //✅ 3. Reset the flag new_color to indicate that the color has been set
    p_fsm_display->new_color = false;
    //✅ 4. Set the display system to idle. In this case, the display system is active, but while the distance is not changed,
    //the display system is idle and can enter in a low power mode
    p_fsm_display->idle = true;
}

/**
 * @brief turn the display system OFF
 * 
 * @param p_this 
 */
static void do_set_off(fsm_t *p_this)
{
    //1. Call function port_display_set_rgb() with the RGB LED ID with no color (COLOR_OFF).
    fsm_display_t *p_fsm_display = (fsm_display_t *)p_this;
    port_display_set_rgb(p_fsm_display->display_id, COLOR_OFF);
    //2. Reset the flag idle to indicate that the display system is not idle
    p_fsm_display->idle = false;
}

static fsm_trans_t fsm_trans_display[] = {
    {WAIT_DISPLAY, check_active, SET_DISPLAY, do_set_on},
    {SET_DISPLAY, check_set_new_color,SET_DISPLAY, do_set_color},
    {SET_DISPLAY, check_off, WAIT_DISPLAY, do_set_off},
    {-1, NULL, -1, NULL} // End of the FSM
};

/* Other auxiliary functions */

/**
 * @brief Iniatialize the FSM display
 * This function initializes the default values of the FSM struct and calls to the port to initialize the associated HW given the ID.
 * The FSM stores the display level of the display system. The user should set it using the function fsm_display_set_distance().
 * 
 * @warning This display system is agnostic to the ultrasound sensor or any other sensor. It only shows the status of the display system set by the user. It does not matter if the display is for a parking sensor, a door sensor, or any other sensor. The display system only shows a status according to a distance set by the user.
 * The FSM contains information of the RGB LED ID. This ID is a unique identifier that is managed by the user in the port.
 *  That is where the user provides identifiers and HW information for all the RGB LEDs on his system. The FSM does not have to know anything of the underlying HW.
 * @param p_fsm_display 
 * @param display_id 
 */
static void fsm_display_init(fsm_display_t *p_fsm_display, uint32_t display_id)
{
    //1. Call the fsm_init() to initialize the FSM. Pass the address of the fsm_t struct and the transition table.
    fsm_init(&p_fsm_display->f, fsm_trans_display);
    //2. Initialize the distance_id
    p_fsm_display->display_id = display_id;
    //3. Set thedistance_cm to-1 or any other invalid value in the range of the distance. 
    p_fsm_display->distance_cm = -1;
    //4. Initialize the flagsnew_color, status, and idle to false
    p_fsm_display->new_color = false;
    p_fsm_display->status = false;
    p_fsm_display->idle = false;
    //5. Call function port_display_init()` to initialize the HW.
    port_display_init(display_id);
}

/* Public functions -----------------------------------------------------------*/
fsm_display_t *fsm_display_new(uint32_t display_id)
{
    fsm_display_t *p_fsm_display = malloc(sizeof(fsm_display_t)); /* Do malloc to reserve memory of all other FSM elements, although it is interpreted as fsm_t (the first element of the structure) */
    fsm_display_init(p_fsm_display, display_id); /* Initialize the FSM */
    return p_fsm_display;
}


void fsm_display_fire(fsm_display_t *p_fsm)
{
    // Call the fire function of the FSM
    fsm_fire(&p_fsm->f);
   
}


void fsm_display_destroy(fsm_display_t *p_fsm)
{
    free(p_fsm); // Free the memory of the FSM
}

fsm_t *fsm_display_get_inner_fsm(fsm_display_t *p_fsm)
{
    //1. Return the address of the f field of the struct.
    return &p_fsm->f;
}

uint32_t fsm_display_get_state(fsm_display_t *p_fsm)
{
    //1. Call function fsm_get_state() with the address of the f field of the struct and return the result.
    return fsm_get_state(&p_fsm->f);
}

void fsm_display_set_distance(fsm_display_t *p_fsm, uint32_t distance_cm)
{
    //1. Set the field distance_cm with the new value.
    p_fsm->distance_cm = distance_cm;
    //2. Set the field new_color to true to indicate that a new color has to be set.
    p_fsm->new_color = true;
}

bool fsm_display_get_status(fsm_display_t *p_fsm)
{
    //1. Return the field status.
    return p_fsm->status;
}

void fsm_display_set_status(fsm_display_t *p_fsm, bool pause)
{
    //1. Update the field status with the new value.
    p_fsm->status = pause;
}

void fsm_display_set_state(fsm_display_t *p_fsm, int8_t state)
{
    //1. Call function fsm_set_state() with the address of the f field of the struct and the new state.
    fsm_set_state(&p_fsm->f, state);
}

bool fsm_display_check_activity(fsm_display_t *p_fsm)
{
    //Return true if the display system is active and it is not idle. Otherwise, return false.
    return (p_fsm->status && !p_fsm->idle); // Display system is active and not idle
}



