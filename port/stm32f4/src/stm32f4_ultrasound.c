/**
 * @file stm32f4_ultrasound.c
 * @brief Portable functions to interact with the ultrasound FSM library. All portable functions must be implemented in this file.
 * @author alumno1
 * @author alumno2
 * @date date
 */

/* Standard C includes */

#include <stdio.h>
#include <math.h>



/* HW dependent includes */

#include "port_system.h"
#include "stm32f4_system.h"
#include "stm32f4_ultrasound.h"
#include "port_ultrasound.h"


/* Microcontroller dependent includes */

/* Typedefs --------------------------------------------------------------------*/

typedef struct 
{
    GPIO_TypeDef *p_trigger_port;
    GPIO_TypeDef *p_echo_port;
    uint8_t trigger_pin;
    uint8_t echo_pin;
    uint8_t echo_alt_fun;
    bool trigger_ready;
    bool trigger_end;
    bool echo_received;
    uint32_t echo_init_tick;
    uint32_t echo_end_tick;
    uint32_t echo_overflows;
} stm32f4_ultrasound_hw_t; //name of the struct 


/* Global variables */

static stm32f4_ultrasound_hw_t ultrasound_arr[] = {
    [PORT_REAR_PARKING_SENSOR_ID] = {
    .p_trigger_port = STM32F4_REAR_PARKING_SENSOR_TRIGGER_GPIO, 
    .trigger_pin = STM32F4_REAR_PARKING_SENSOR_TRIGGER_PIN,
    .p_echo_port = STM32F4_REAR_PARKING_SENSOR_ECHO_GPIO,
    .echo_pin = STM32F4_REAR_PARKING_SENSOR_ECHO_PIN,
    .echo_alt_fun = 0,
    .trigger_ready = false,
    .trigger_end = false,
    .echo_received = false,
    .echo_init_tick = 0,
    .echo_end_tick = 0,
    .echo_overflows = 0},
};

/* Private functions ----------------------------------------------------------*/

stm32f4_ultrasound_hw_t *_stm32f4_ultrasound_get(uint32_t ultrasound_id)
{   
    // Return the pointer to the ultrasound with the given ID. If the ID is not valid, return NULL.
    if (ultrasound_id < sizeof(ultrasound_arr) / sizeof(ultrasound_arr[0]))
    {
        return &ultrasound_arr[ultrasound_id];
    }
    else
    {
        return NULL;
    }
}
/**
 * @brief Configure the timer that controls the duration of the trigger signal.
 * This function configures the timer to generate internal interrupts to control the raise and fall of the trigger signal. 
 * The duration of the trigger signal is defined in the PORT_PARKING_SENSOR_TRIGGER_UP_US macro.
 * This function is called by the port_ultrasound_init() public function to configure the timer that controls the duration of the trigger signal.
 */
static void _timer_trigger_setup() //TRIGGER TIMER3  // Analizadlo/entenderlo otro dia, estamos muertos
{
    // Enable the clock of the timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // datasheet we use timer 2 & 3 so its conneted to APB1

    // Disable the counter of the timer
    TIM3->CR1 &= ~TIM_CR1_CEN;  /* TIM_CR1_CEN is !<Counter enable */

    // Enable the autoreload preload
    TIM3->CR1 |= TIM_CR1_ARPE;  /* TIM_CR1_ARPE is !<Auto-reload preload enable, con esto, al activar ARPE, los cambios en ARR solo funcionan cuando el temporizador se reincia */

    // Set the counter to 0
    TIM3->CNT = 0;

    // compute the prescaler and the auto reload register to set the duration of the trigger signal
    double system_clock = (double)SystemCoreClock;
    double trigger_duration_us = (double)PORT_PARKING_SENSOR_TRIGGER_UP_US; // 10us
    double trigger_duration_s = trigger_duration_us * 1e-6; //also is valid to use trigger_duration_us / 1e6
    
    double psc = round((system_clock * trigger_duration_s / 65535.0) - 1.0); // Compute an initial value for the PSC register considering the maximum value of ARR (65535.0)
    double arr = round((system_clock * trigger_duration_s / (psc + 1.0)) - 1.0); // Compute an initial value for the ARR register

    if (arr > 65535.0) //NOTE: 65535 is like 0xFFFF
    {
        psc += 1.0; // Increment the PSC register to reduce the ARR value
        arr = round((system_clock * trigger_duration_s / (psc + 1.0)) - 1.0);
    }
     
    // Set the prescaler and the auto reload register
    TIM3->PSC = (uint32_t)psc;
    TIM3->ARR = (uint32_t)arr;


    // Generate an update event to load the PSC and ARR values into the active registers
    //We achieve this by setting the UG bit of the EGR register.
    TIM3->EGR |= TIM_EGR_UG;  /*!<Update Generation                         */

    // Clear the update interrupt flag,  Clear the update interrupt flag (bit UIF of the register SR) to avoid an unwanted interrupt.
    TIM3->SR &= ~TIM_SR_UIF;    /*!<Update interrupt Flag              */

    // Enable the update interrupt, Enable the interrupts of the timer by setting the UIE bit of the DIER register.
    TIM3->DIER |= TIM_DIER_UIE;    /*!<Update interrupt enable */

    // Set the priority of the timer interrupt in the NVIC
    NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 4, 0)); //priority 4, subpriority 0
}

/* Public functions -----------------------------------------------------------*/

/**
 * @brief Configure the HW specifications of a given ultrasound sensor.
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_init(uint32_t ultrasound_id)
{
    /* Get the ultrasound sensor */
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);

    /* TO-DO alumnos: */
    // Initialize the fields of the ultrasound struct
    p_ultrasound->echo_init_tick = 0;
    p_ultrasound->echo_end_tick = 0;
    p_ultrasound->echo_overflows = 0;
    p_ultrasound->trigger_ready = true; 
    p_ultrasound->trigger_end = false;
    p_ultrasound->echo_received = false;

    /* Trigger pin configuration */ 
    //configure the trigger pin as output and no pull-up neither pull-down conection
    stm32f4_system_gpio_config(p_ultrasound->p_trigger_port, p_ultrasound->trigger_pin, MODER13_AS_OUTPUT, STM32F4_GPIO_PUPDR_NOPULL); //MODER13_AS_OUTPUT is 0x01
    
    /* Echo pin configuration */
    stm32f4_system_gpio_config(p_ultrasound->p_echo_port, p_ultrasound->echo_pin, MODER13_AS_ALTERNATE, STM32F4_GPIO_PUPDR_NOPULL); //MODER13_AS_ALTERNATE is 0x02
    
    /* Configure timers */
    _timer_trigger_setup();
    //_timer_echo_setup();
    //_timer_new_measurement_setup();
}
/**
 * @brief Stop the timer that controls the trigger signal.
 *This function stops the timer that controls the trigger signal because the time to trigger the ultrasound sensor has finished.
 *It also sets the trigger signal to low.
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_stop_trigger_timmer(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    // Set the trigger pin to low
    stm32f4_system_gpio_write(p_ultrasound->p_trigger_port, p_ultrasound->trigger_pin, 0);

    // Disable the trigger timer
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

// Getters and setters functions

/**
 * @brief Get the readiness of the trigger signal.
 * This function returns the status of readiness the trigger signal. If it is true, the ultrasound sensor is ready to start a new measurement.
 * 
 * @param ultrasound_id 
 * @return true 
 * @return false 
 */
bool port_ultrasound_get_trigger_ready(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->trigger_ready;
}

/**
 * @brief Get the status of the trigger signal.
 * This function returns the status of the trigger signal. It will be true if the time to trigger the ultrasound sensor has finished.
 * 
 * @param ultrasound_id 
 * @return true 
 * @return false 
 */
bool port_ultrasound_get_trigger_end(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->trigger_end;
}

/**
 * @brief Set the readiness of the trigger signal.
 * This function sets the status of readiness of the trigger signal.
 * If it is true, the ultrasound sensor will ready to start a new measurement. 
 * 
 * @param ultrasound_id 
 * @param trigger_ready 
 */
void port_ultrasound_set_trigger_ready(uint32_t ultrasound_id, bool trigger_ready)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->trigger_ready = trigger_ready;
}

/**
 * @brief Set the status of the trigger signal.
 * This function sets the status of the trigger signal.
 * It will be true to indicate that the time to trigger the ultrasound sensor has finished and the trigger signal is low.
 * 
 * @param ultrasound_id 
 * @param trigger_end 
 */
void port_ultrasound_set_trigger_end(uint32_t ultrasound_id, bool trigger_end)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->trigger_end = trigger_end;
}









// Util
void stm32f4_ultrasound_set_new_trigger_gpio(uint32_t ultrasound_id, GPIO_TypeDef *p_port, uint8_t pin)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->p_trigger_port = p_port;
    p_ultrasound->trigger_pin = pin;
}

void stm32f4_ultrasound_set_new_echo_gpio(uint32_t ultrasound_id, GPIO_TypeDef *p_port, uint8_t pin)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->p_echo_port = p_port;
    p_ultrasound->echo_pin = pin;
}