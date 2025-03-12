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
    [PORT_REAR_PARKING_SENSOR_ID] = {.p_trigger_port = STM32F4_REAR_PARKING_SENSOR_TRIGGER_GPIO, 
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

static void _timer_trigger_setup() //TRIGGER TIMER3  // Analizadlo/entenderlo otro dia, estamos muertos
{
    // Enable the clock of the timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; // datasheet we use timer 2 & 3 so its conneted to APB1

    // Disable the counter of the timer
    TIM3->CR1 &= ~TIM_CR1_CEN;

    // Enable the autoreload preload
    TIM3->CR1 |= TIM_CR1_ARPE;

    // Set the counter to 0
    TIM3->CNT = 0;

    // compute the prescaler and the auto reload register to set the duration of the trigger signal
    double system_clock = (double)SystemCoreClock;
    double trigger_duration_us = (double)PORT_PARKING_SENSOR_TRIGGER_UP_US;
    double psc = round(system_clock * trigger_duration_us / 1e6 / 65535.0 - 1.0);
    double arr = round(system_clock * trigger_duration_us / 1e6 / (psc + 1.0) - 1.0);

    if (arr > 65535.0)
    {
        psc += 1.0;
        arr = round(system_clock * trigger_duration_us / 1e6 / (psc + 1.0) - 1.0);
    }

    TIM3->PSC = (uint32_t)psc;
    TIM3->ARR = (uint32_t)arr;


    // Generate an update event to load the PSC and ARR values into the active registers
    TIM3->EGR |= TIM_EGR_UG;

    // Clear the update interrupt flag
    TIM3->SR &= ~TIM_SR_UIF;

    // Enable the update interrupt
    TIM3->DIER |= TIM_DIER_UIE;

    // Set the priority of the timer interrupt in the NVIC
    NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
}

/* Public functions -----------------------------------------------------------*/
void port_ultrasound_init(uint32_t ultrasound_id)
{
    /* Get the ultrasound sensor */
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);

    /* TO-DO alumnos: */

    /* Trigger pin configuration */ 
    //configure the trigger pin as output and no pull-up neither pull-down conection
    stm32f4_gpio_pin_cfg(p_ultrasound->p_trigger_port, p_ultrasound->trigger_pin, );



    /* Echo pin configuration */

    /* Configure timers */
}

// Getters and setters functions


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