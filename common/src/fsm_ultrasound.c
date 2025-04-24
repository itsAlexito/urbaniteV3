/**
 * @file fsm_ultrasound.c
 * @brief Ultrasound sensor FSM main file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

/* Includes ------------------------------------------------------------------*/
/* Standard C includes */
#include <stdlib.h>
#include <string.h>
#include "port_ultrasound.h"
#include "port_system.h"
#include "fsm.h"
#include "fsm_ultrasound.h"


/* HW dependent includes */

/* Project includes */

/* Struct */
struct fsm_ultrasound_t
{
    fsm_t f; /*!< Ultrasound FSM */
    uint32_t distance_cm; /*!< Distance measured by the ultrasound sensor */
    bool status; /*!< Status of the ultrasound sensor */
    bool new_measurement; /*!< Flag indicating if a new measurement is available */
    uint32_t ultrasound_id; /*!< ID of the ultrasound sensor, must be unique */
    uint32_t distance_arr[FSM_ULTRASOUND_NUM_MEASUREMENTS]; /*!< Array to store the distances measured by the ultrasound sensor */
    uint32_t distance_idx; /*!< Index of the distance array */
};
/* Typedefs --------------------------------------------------------------------*/

/* Private functions -----------------------------------------------------------*/
// Comparison function for qsort
int _compare(const void *a, const void *b)
{
    return (*(uint32_t *)a - *(uint32_t *)b);
}

/* State machine input or transition functions */

/**
 * @brief Check if the ultrasound sensor is active and ready to start a new measurement.
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_on(fsm_t *p_this)
{
    return port_ultrasound_get_trigger_ready(((fsm_ultrasound_t *)p_this)->ultrasound_id);
}

static bool check_off(fsm_t *p_this)
{
    fsm_ultrasound_t *p_fsm_ultrasound = (fsm_ultrasound_t *)p_this;
    return !p_fsm_ultrasound->status;
}
/**
 * @brief Check if the trigger signal has ended. Call function port_ultrasound_get_trigger_end and return the value
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_trigger_end(fsm_t *p_this)
{
    fsm_ultrasound_t *p_fsm_ultrasound = (fsm_ultrasound_t *)p_this;
    return port_ultrasound_get_trigger_end(p_fsm_ultrasound->ultrasound_id);
    //return port_ultrasound_get_trigger_end(((fsm_ultrasound_t *)p_this)->ultrasound_id);
}

/**
 * @brief Check if the echo signal has received the init (rising edge). Call function port_ultrasound_get_echo_init_tick and return the value
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_echo_init(fsm_t *p_this)
{
    fsm_ultrasound_t *p_fsm_ultrasound = (fsm_ultrasound_t *)p_this;

    uint32_t echo_init_tick = port_ultrasound_get_echo_init_tick(p_fsm_ultrasound->ultrasound_id);

    return (echo_init_tick >0);
}
/**
 * @brief check if the ultrasound sensor has received the en (falling edge in the input capture) of the echo signal
 * . Call function port_ultrasound_get_echo_end_tick and return the value
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_echo_received(fsm_t *p_this)
{
    return port_ultrasound_get_echo_received(((fsm_ultrasound_t *)p_this)->ultrasound_id);
}

/**
 * @brief check if a new measurement is ready. 
 * Call function port_ultrasound_get_new_measurement_ready and return the value
 * 
 * @param p_this 
 * @return true 
 * @return false 
 */
static bool check_new_measurement(fsm_t *p_this)
{
    return port_ultrasound_get_trigger_ready(((fsm_ultrasound_t *)p_this)->ultrasound_id);
}


/* --------------------------------- State machine output or action functions -------------------------------*/
/**
 * @brief start a measurement of the ultrasound tranceiver for the first time after the FSM is started.
 * 
 * @param p_this 
 */
static void do_start_measurement(fsm_t *p_this)
{
    // Convertir el puntero genérico de la FSM al tipo específico fsm_ultrasound_t
    fsm_ultrasound_t *p_fsm_ultrasound = (fsm_ultrasound_t *)p_this;


    // Llamar a la función de inicio de medición del puerto
    port_ultrasound_start_measurement(p_fsm_ultrasound->ultrasound_id);
}
/**
 * @brief Stop the trigger signal of the ultrasound sensor.
 * 
 * 
 * @param p_this 
 */
static void do_stop_trigger(fsm_t *p_this)
{
    port_ultrasound_stop_trigger_timer(((fsm_ultrasound_t *)p_this)->ultrasound_id);
    port_ultrasound_set_trigger_end(((fsm_ultrasound_t *)p_this)->ultrasound_id, false);
}


static void do_set_distance(fsm_t *p_this)
{
    fsm_ultrasound_t *p_fsm = (fsm_ultrasound_t *)p_this;

    // 1. Retrieve echo ticks
    uint32_t echo_init_tick = port_ultrasound_get_echo_init_tick(p_fsm->ultrasound_id);
    uint32_t echo_end_tick = port_ultrasound_get_echo_end_tick(p_fsm->ultrasound_id);
    uint32_t echo_overflows = port_ultrasound_get_echo_overflows(p_fsm->ultrasound_id);

    // 2. Calculate total ticks (including overflows)
    uint32_t total_time_ticks = (echo_end_tick + echo_overflows * (0xFFFF + 1)) - echo_init_tick;

    // 3. Calculate distance (cm)
    uint32_t distance_cm = (total_time_ticks * 34300) / (2 * 1000000);

    // 4. Store distance in array at current index
    p_fsm->distance_arr[p_fsm->distance_idx] = distance_cm;

    // 5. If buffer full (last slot)
    if (p_fsm->distance_idx == FSM_ULTRASOUND_NUM_MEASUREMENTS - 1)
    {
        // 5a. Sort array
        qsort(p_fsm->distance_arr, FSM_ULTRASOUND_NUM_MEASUREMENTS, sizeof(uint32_t), _compare);

        // 6. Compute median
        if (FSM_ULTRASOUND_NUM_MEASUREMENTS % 2 == 0) // Even
        {
            p_fsm->distance_cm = (p_fsm->distance_arr[FSM_ULTRASOUND_NUM_MEASUREMENTS / 2 - 1] +
                                  p_fsm->distance_arr[FSM_ULTRASOUND_NUM_MEASUREMENTS / 2]) / 2;
        }
        else // Odd
        {
            p_fsm->distance_cm = p_fsm->distance_arr[FSM_ULTRASOUND_NUM_MEASUREMENTS / 2];
        }

        // 7. Set new measurement flag
        p_fsm->new_measurement = true;
    }

    // 8. Increment index
    p_fsm->distance_idx = (p_fsm->distance_idx + 1) % FSM_ULTRASOUND_NUM_MEASUREMENTS;

    // 9. Stop the echo timer
    port_ultrasound_stop_echo_timer(p_fsm->ultrasound_id);

    // 10. Reset the echo ticks
    port_ultrasound_reset_echo_ticks(p_fsm->ultrasound_id);
}

/**
 * @brief 
 * 
 * @param p_this 
 */
static void do_stop_measurement(fsm_t *p_this){
   //call function port_ultrasound_stop_ultrasound to stop the ultrasound sensor
   port_ultrasound_stop_ultrasound(((fsm_ultrasound_t *)p_this)->ultrasound_id);
}

/**
 * @brief 
 * 
 * @param p_this 
 */
static void do_start_new_measurement(fsm_t *p_this)
{
    //call function port_ultrasound_start_new_measurement to start a new measurement
    do_start_measurement(p_this);
}

static fsm_trans_t fsm_trans_ultrasound[] = {
    {WAIT_START, check_on, TRIGGER_START, do_start_measurement},
    {TRIGGER_START, check_trigger_end, WAIT_ECHO_START, do_stop_trigger},
    {WAIT_ECHO_START, check_echo_init, WAIT_ECHO_END, NULL},
    {WAIT_ECHO_END, check_echo_received, SET_DISTANCE, do_set_distance},
    {SET_DISTANCE, check_new_measurement, TRIGGER_START, do_start_new_measurement},
    {SET_DISTANCE, check_off, WAIT_START, do_stop_measurement},  
    {-1, NULL, -1, NULL} // End of the FSM
};

/* --------------------Other auxiliary functions--------------- */

/**
 * @brief Initialize the FSM structure.
 * This function initializies the default values of the FSM struct and calls to the port to initialize the associated HW given the ID.
 * The FSM stores the distance of the last ultrasound trigger.
 * The user should ask for it using the function fsm_ultrasound_get_distance().
 * The FSM contains information of the ultrasound ID. This ID is a unique identifier that is managed by the user in the port.
 * That is where the user provides identifiers and HW information for all the ultrasounds on his system. 
 * The FSM does not have to know anything of the underlying HW.
 * 
 * @param p_fsm_ultrasound 
 * @param ultrasound_id 
 */
void fsm_ultrasound_init(fsm_ultrasound_t *p_fsm_ultrasound, uint32_t ultrasound_id)
{
    // Iniatilize the FSM
    fsm_init(&p_fsm_ultrasound->f, fsm_trans_ultrasound);

        // 1. Limpiar la estructura completa
        p_fsm_ultrasound->ultrasound_id = ultrasound_id;
        p_fsm_ultrasound->distance_cm = 0;
        p_fsm_ultrasound->distance_idx = 0;
        memset(p_fsm_ultrasound->distance_arr, 0, sizeof(p_fsm_ultrasound->distance_arr));

        p_fsm_ultrasound->status = false;
        p_fsm_ultrasound->new_measurement = false;


    // 5. Inicializar el HW del sensor
    port_ultrasound_init(ultrasound_id);
}


/* Public functions -----------------------------------------------------------*/
fsm_ultrasound_t *fsm_ultrasound_new(uint32_t ultrasound_id)
{
    fsm_ultrasound_t *p_fsm_ultrasound = malloc(sizeof(fsm_ultrasound_t));/* Do malloc to reserve memory of all other FSM elements, although it is interpreted as fsm_t (the first element of the structure) */
    fsm_ultrasound_init(p_fsm_ultrasound, ultrasound_id);                  /* Initialize the FSM */
    return p_fsm_ultrasound;
}



/* FSM-interface functions. These functions are used to interact with the FSM */

void fsm_ultrasound_fire(fsm_ultrasound_t *p_fsm)
{
    fsm_fire(&p_fsm->f); // Call the fire function of the FSM
}

void fsm_ultrasound_destroy(fsm_ultrasound_t *p_fsm)
{
    free(p_fsm); // Free the memory of the FSM
}

fsm_t *fsm_ultrasound_get_inner_fsm(fsm_ultrasound_t *p_fsm)
{
    //1. Return the address of the f field of the struct.
    return &p_fsm->f;
}

uint32_t fsm_ultrasound_get_state(fsm_ultrasound_t *p_fsm)
{
    //1.Retrieve and return the field current_state of the FSM (field f of the struct).
    return p_fsm->f.current_state;
}

uint32_t fsm_ultrasound_get_distance(fsm_ultrasound_t *p_fsm)
{
    p_fsm->new_measurement = false;
    // 1. Retrieve and return the field distance_cm.
    return p_fsm->distance_cm;
    // 2. Reset the field new_measurement.
}

void fsm_ultrasound_stop(fsm_ultrasound_t *p_fsm)
{
    //1. Reset the field status.
    p_fsm->status = false;
    //2.  Call function port_ultrasound_stop_ultrasound() with the right parameters.
    port_ultrasound_stop_ultrasound(p_fsm->ultrasound_id);
}

void fsm_ultrasound_start(fsm_ultrasound_t *p_fsm)
{
    //1. Set the field status to true.
    p_fsm->status = true;
    //2. Reset the field distance_idx to 0.
    p_fsm->distance_idx = 0;
    //3. Reset the field distance_cm to 0.
    p_fsm->distance_cm = 0;
    //4. Call function port_ultrasound_reset_echo_ticks() with the right parameters.
    port_ultrasound_reset_echo_ticks(p_fsm->ultrasound_id);
    //5. 5. Call function port_ultrasound_set_trigger_ready() with the right parameters to indicate that the ultrasound sensor is ready to start a new measurement.
    port_ultrasound_set_trigger_ready(p_fsm->ultrasound_id, true);
    //6. Call function port_ultrasound_start_new_measurement_timer() to force the new measurement timer to start to provoke the first interrupt.
    port_ultrasound_start_new_measurement_timer();
}

bool fsm_ultrasound_get_status(fsm_ultrasound_t *p_fsm)
{
    //1. Return the field status.
    return p_fsm->status;
}

void fsm_ultrasound_set_status(fsm_ultrasound_t *p_fsm, bool status)
{
    //1. Update the field status with the new value.
    p_fsm->status = status;
}

bool fsm_ultrasound_get_ready(fsm_ultrasound_t *p_fsm)
{
    //1. Call function port_ultrasound_get_trigger_ready() with the ultrasound ID and return the result.
    return port_ultrasound_get_trigger_ready(p_fsm->ultrasound_id);
}

bool fsm_ultrasound_get_new_measurement_ready(fsm_ultrasound_t *p_fsm)
{
    //1. Retrieve and return the field new_measurement.
    return p_fsm->new_measurement;
}

//-------------------v4------------------------------------------------
bool fsm_ultrasound_check_activity(fsm_ultrasound_t *p_fsm)
{
    //✅ 1. Return false always.
    return false;
}

// ------------------Other auxiliary functions------------------------
void fsm_ultrasound_set_state(fsm_ultrasound_t *p_fsm, int8_t state)
{
    p_fsm->f.current_state = state;
}