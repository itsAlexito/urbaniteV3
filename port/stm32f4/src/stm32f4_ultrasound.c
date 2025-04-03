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

/**
 * @brief Structure for the ultrasound hardware information.
 * 
 */
typedef struct 
{
    GPIO_TypeDef *p_trigger_port; /*!< Pointer to the GPIO port used for the trigger pin */
    GPIO_TypeDef *p_echo_port; /*!< Pointer to the GPIO port used for the echo pin */
    uint8_t trigger_pin; /*!< Pin number of the trigger pin */
    uint8_t echo_pin; /*!< Pin number of the echo pin */
    uint8_t echo_alt_fun; /*!< Alternate function number for the echo pin */
    bool trigger_ready; /*!< Flag indicating whether the trigger is ready to be sent */
    bool trigger_end; /*!< Flag indicating whether the trigger has ended */
    bool echo_received; /*!< Flag indicating whether the echo signal has been received */
    uint32_t echo_init_tick;/*!< Initial tick count when the echo signal is received */
    uint32_t echo_end_tick; /*!< Final tick count when the echo signal ends */
    uint32_t echo_overflows;  /*!< Number of overflows that occurred while waiting for the echo signal */
} stm32f4_ultrasound_hw_t; //name of the struct 


/* Global variables */

/**
 * @brief Structure to define the HW dependencies of aN ultrasound sensor.
 * 
 */
static stm32f4_ultrasound_hw_t ultrasound_arr[] = {
    [PORT_REAR_PARKING_SENSOR_ID] = {
    .p_trigger_port = STM32F4_REAR_PARKING_SENSOR_TRIGGER_GPIO, 
    .trigger_pin = STM32F4_REAR_PARKING_SENSOR_TRIGGER_PIN,
    .p_echo_port = STM32F4_REAR_PARKING_SENSOR_ECHO_GPIO,
    .echo_pin = STM32F4_REAR_PARKING_SENSOR_ECHO_PIN,
    .echo_alt_fun = STM32F4_AF1, /*!<datasheet says that the echo signal is connected to TIM2_CH1, so we use AF1*/
    .trigger_ready = false,
    .trigger_end = false,
    .echo_received = false,
    .echo_init_tick = 0,
    .echo_end_tick = 0,
    .echo_overflows = 0},
};

/* Private functions ----------------------------------------------------------*/

/**
 * @brief Structure to define the HW dependencies of aN ultrasound sensor.
 * 
 * @param ultrasound_id 
 * @return stm32f4_ultrasound_hw_t* 
 */
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
static void _timer_trigger_setup() //TRIGGER TIMER3  
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

/**
 * @brief Configure the timer that controls the duration of the echo signal.
 * 
 * @param ultrasound_id 
 */
static void _timer_echo_setup(uint32_t ultrasound_id) //ECHO = TIMER 2
{
    // Enable the clock of the timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // datasheet we use timer 2 & 3 so its conneted to APB1

    TIM2->CR1 = 0x0000 ; /* Aunque es el valor por defecto */
    TIM2->CR1 &= ~TIM_CR1_CEN ; /* Deshabilita el contador*/
    TIM2->CNT = 0; /* Aseguramos que el contador esta a 0 */



    /*✅ 2. Set the values of the prescaler and the auto-reload registers. 
    Configure PSC for a resolution of 1 us (1 MHz) and ARR high enough to avoid too many overflows, e.g. it maximum value.
    */
    double system_clock = (double)SystemCoreClock;
    double Tclk = 1*1e-6; // 1 us resolution
    double arr = 0xFFFF; // Maximum value of ARR
    double psc = round(((system_clock*Tclk))- 1); // 1 us resolution, formula from the book

    // Set the prescaler and the auto reload register
    TIM2->PSC = (uint32_t)psc;
    TIM2->ARR = (uint32_t)arr;

    /*✅ 3. Set the auto-reload preload bit (ARPE) in the control register (CR1) to enable the auto-reload register.
    Also enable the update generation bit (UG) in the event generation register (EGR) to force the update of the registers.
    */
    TIM2->CR1 |= TIM_CR1_ARPE;  /* TIM_CR1_ARPE is !<Auto-reload preload enable, con esto, al activar ARPE, los cambios en ARR solo funcionan cuando el temporizador se reincia */
    TIM2->EGR |= TIM_EGR_UG;  /*!<Update Generation                         */
    
    /*✅ 4. Set the direction as input in the Capture/Compare mode register (CCMRx).
    x is 1 for channels 1 and 2, and 2 for channels 3 and 4. 
     Check the table "Table 11. Alternate function" in the datasheet to identify the channel related to timer associated to the pin of the echo signal.
    */
    TIM2 -> CCMR1 |= TIM_CCMR1_CC2S_0; //as it uses the pin 1 and TIM2 is connected to channel 2, so we use CCMR1


    /*✅ 5. Disable digital filtering by clearing the ICxF bits in the Capture/Compare mode register (CCMRx).
    */
    TIM2 -> CCMR1 &= ~TIM_CCMR1_IC2F; //IC2F is 0x0, so we clear it, /* Limpiamos para asegurar que esta a 0 */

    /*✅ 6. Select the edge of the active transition in the Capture/Compare enable register (CCER).
     Set the bits CCxNP and CCxP to detect both rising and falling edges.
    */
    TIM2 -> CCER |= (1 << TIM_CCER_CC2P_Pos | 1 << TIM_CCER_CC2NP_Pos ) ; /* Rising edge */
    /*✅ 7. Program the input prescaler to capture each valid transition.
     Set the ICxPSC bits in the Capture/Compare mode register (CCMRx) to 0.
    */
    TIM2 -> CCMR1 &= ~TIM_CCMR1_IC2PSC; //Prescaler de entrada . Para capturar cada transicion valida , poner a 0 */

    /*✅ 8. Enable the Capture/compare enable register (CCER) for the corresponding channel.
    */
    TIM2 -> CCER |= TIM_CCER_CC2E; /*!<Capture/Compare 1 output enable ,  */

    /*✅ 9. Enable the Capture/Compare interrupts bit (CCxIE) for the corresponding channel in the DMA/interrupt enable register (DIER).
    */

    TIM2 -> CCER |= TIM_CCER_CC2E ; /*!<Capture/Compare 1 interrupt enable ,  *//* Interrumpe al capturar */

    /*✅ 10. Enable the update interrupt bit (UIE) in the DMA/interrupt enable register (DIER).
    */
    TIM2 -> DIER |= TIM_DIER_UIE; /*!<Update interrupt enable ,  *//* Interrumpe al actualizar */
    TIM2 -> DIER |= TIM_DIER_CC2IE ; /* Interrumpe al capturar */

    /*✅ 11. Set the priority of the timer interrupt in the NVIC using the NVIC_SetPriority() function and the TIMx_IRQn interrupt
     with the level of priority and sub-priority shown in the main page.
    */
    NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 3, 0)); //priority 3, subpriority 0
}

/**
 * @brief Configure the timer that controls the duration of the new measurement.
 * 
 */
void _timer_new_measurement_setup(){

        // Enable the clock of the timer
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN; // datasheet we use timer 2 & 3 so its conneted to APB1

        // Disable the counter of the timer
        TIM5->CR1 &= ~TIM_CR1_CEN;  /* TIM_CR1_CEN is !<Counter enable */
    
        // Enable the autoreload preload
        TIM5->CR1 |= TIM_CR1_ARPE;  /* TIM_CR1_ARPE is !<Auto-reload preload enable, con esto, al activar ARPE, los cambios en ARR solo funcionan cuando el temporizador se reincia */
    
        // Set the counter to 0
        TIM5->CNT = 0;
    
        // compute the prescaler and the auto reload register to set the duration of the trigger signal
        double system_clock = (double)SystemCoreClock;
        double trigger_duration_ms = (double)PORT_PARKING_SENSOR_TIMEOUT_MS; // 100 ms
        double trigger_duration_s = trigger_duration_ms * 1e-3; //also is valid to use trigger_duration_us / 1e6
        
        double psc = round((system_clock * trigger_duration_s / 65535.0) - 1.0); // Compute an initial value for the PSC register considering the maximum value of ARR (65535.0)
        double arr = round((system_clock * trigger_duration_s / (psc + 1.0)) - 1.0); // Compute an initial value for the ARR register
    
        if (arr > 65535.0) //NOTE: 65535 is like 0xFFFF
        {
            psc += 1.0; // Increment the PSC register to reduce the ARR value
            arr = round((system_clock * trigger_duration_s / (psc + 1.0)) - 1.0);
        }
         
        // Set the prescaler and the auto reload register
        TIM5->PSC = (uint32_t)psc;
        TIM5->ARR = (uint32_t)arr;
    
    
        // Generate an update event to load the PSC and ARR values into the active registers
        //We achieve this by setting the UG bit of the EGR register.
        TIM5->EGR |= TIM_EGR_UG;  /*!<Update Generation                         */
    
        // Clear the update interrupt flag,  Clear the update interrupt flag (bit UIF of the register SR) to avoid an unwanted interrupt.
        TIM5->SR &= ~TIM_SR_UIF;    /*!<Update interrupt Flag              */
    
        // Enable the update interrupt, Enable the interrupts of the timer by setting the UIE bit of the DIER register.
        TIM5->DIER |= TIM_DIER_UIE;    /*!<Update interrupt enable */
    
        // Set the priority of the timer interrupt in the NVIC
        NVIC_SetPriority(TIM5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0)); //priority 5, subpriority 0
}



/* Public functions -----------------------------------------------------------*/


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
    stm32f4_system_gpio_config(p_ultrasound->p_trigger_port, p_ultrasound->trigger_pin,STM32F4_GPIO_MODE_OUT, STM32F4_GPIO_PUPDR_NOPULL); //MODER0_AS_OUTPUT is 0x01

    /* Echo pin configuration */ 
    stm32f4_system_gpio_config(p_ultrasound->p_echo_port, p_ultrasound->echo_pin, STM32F4_GPIO_MODE_AF, STM32F4_GPIO_PUPDR_NOPULL);
    stm32f4_system_gpio_config_alternate(p_ultrasound->p_echo_port, p_ultrasound->echo_pin, STM32F4_AF1); //AF1 segun el datsheet corresponde a TMR2/
    /* Configure timers */
    _timer_trigger_setup();
    _timer_echo_setup(ultrasound_id);
    _timer_new_measurement_setup();
}


void port_ultrasound_stop_trigger_timer(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    // Set the trigger pin to low
    stm32f4_system_gpio_write(p_ultrasound->p_trigger_port, p_ultrasound->trigger_pin, 0);

    // Disable the trigger timer
    TIM3->CR1 &= ~TIM_CR1_CEN;
}


void port_ultrasound_stop_echo_timer(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    // Disable the echo timer
    if (p_ultrasound != NULL)
    {
        TIM2->CR1 &= ~TIM_CR1_CEN;
    }
}


void port_ultrasound_reset_echo_ticks(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->echo_init_tick = 0;
    p_ultrasound->echo_end_tick = 0;
    p_ultrasound->echo_overflows = 0;
    p_ultrasound->echo_received = false;
}

// Getters and setters functions


bool port_ultrasound_get_trigger_ready(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->trigger_ready;
}


bool port_ultrasound_get_trigger_end(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->trigger_end;
}


uint32_t port_ultrasound_get_echo_init_tick(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->echo_init_tick;
}

uint32_t port_ultrasound_get_echo_end_tick(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->echo_end_tick;
}


bool port_ultrasound_get_echo_received(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->echo_received;
}


uint32_t port_ultrasound_get_echo_overflows(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    return p_ultrasound->echo_overflows;
}
//--------------------------------------------------------------------------------------------------------//

void port_ultrasound_set_trigger_ready(uint32_t ultrasound_id, bool trigger_ready)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->trigger_ready = trigger_ready;
}

/**
 * @brief 
 * 
 * @param ultrasound_id 
 * @param trigger_end 
 */
void port_ultrasound_set_trigger_end(uint32_t ultrasound_id, bool trigger_end)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->trigger_end = trigger_end;
}

void port_ultrasound_set_echo_init_tick(uint32_t ultrasound_id, uint32_t echo_init_tick)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->echo_init_tick = echo_init_tick;
}


void port_ultrasound_set_echo_end_tick(uint32_t ultrasound_id, uint32_t echo_end_tick)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->echo_end_tick = echo_end_tick;
}


void port_ultrasound_set_echo_received(uint32_t ultrasound_id, bool echo_received)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->echo_received = echo_received;
}


void port_ultrasound_set_echo_overflows(uint32_t ultrasound_id, uint32_t echo_overflows)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);
    p_ultrasound->echo_overflows = echo_overflows;
}


void port_ultrasound_start_measurement(uint32_t ultrasound_id)
{
    stm32f4_ultrasound_hw_t *p_ultrasound = _stm32f4_ultrasound_get(ultrasound_id);

    // 1. Reset the flag trigger_ready
    port_ultrasound_set_trigger_ready(ultrasound_id, false); 

    // 2. Reset the counters of the timers
    TIM3->CNT = 0; // Trigger timer
    TIM5->CNT = 0; // New measurement timer
    if (p_ultrasound != NULL)
    {
        TIM2->CNT = 0; // Echo timer
    }

    // 3. Set the trigger pin to high
    stm32f4_system_gpio_write(p_ultrasound->p_trigger_port, p_ultrasound->trigger_pin, true);

    // 4. Enable the timers interrupts in the NVIC
    NVIC_EnableIRQ(TIM5_IRQn); // New measurement timer interrupt
    if (p_ultrasound != NULL)
    {
        NVIC_EnableIRQ(TIM3_IRQn); // Trigger timer interrupt
        NVIC_EnableIRQ(TIM2_IRQn); // Echo timer interrupt
    }

  //5. Enable the timers
    TIM3->CR1 |= TIM_CR1_CEN; // Trigger timer
    TIM5->CR1 |= TIM_CR1_CEN; // New measurement timer
    if (p_ultrasound != NULL)
    {
        TIM2->CR1 |= TIM_CR1_CEN; // Echo timer
    }
    
}


void port_ultrasound_start_new_measurement_timer(void)
{
    NVIC_EnableIRQ(TIM5_IRQn); //Enable the interrupt of the new measurement timer in the NVIC
    TIM5->CR1 |= TIM_CR1_CEN; // Enable the new measurement timer
}


void port_ultrasound_stop_new_measurement_timer(void)
{
    TIM5->CR1 &= ~TIM_CR1_CEN; // Disable the new measurement timer
}


void port_ultrasound_stop_ultrasound(uint32_t ultrasound_id)
{
    port_ultrasound_stop_trigger_timer(ultrasound_id); // Stop the trigger timer
    port_ultrasound_stop_echo_timer(ultrasound_id); // Stop the echo timer
    port_ultrasound_stop_new_measurement_timer(); // Stop the new measurement timer
    port_ultrasound_reset_echo_ticks(ultrasound_id); // Reset the echo ticks
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