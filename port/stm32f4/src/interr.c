/**
 * @file interr.c
 * @brief Interrupt service routines for the STM32F4 platform.
 * @author SDG2. Román Cárdenas (r.cardenas@upm.es) and Josué Pagán (j.pagan@upm.es)
 * @date 2025-01-01
 */
// Include HW dependencies:
#include "port_system.h"
#include "stm32f4_system.h"
#include "port_button.h"
#include "stm32f4_button.h"
//includes V2
#include "port_ultrasound.h"
#include "stm32f4_ultrasound.h"
// Include headers of different port elements:

//------------------------------------------------------
// INTERRUPT SERVICE ROUTINES
//------------------------------------------------------
/**
 * @brief Interrupt service routine for the System tick timer (SysTick).
 *
 * @note This ISR is called when the SysTick timer generates an interrupt.
 * The program flow jumps to this ISR and increments the tick counter by one millisecond.
 *
 * > **TO-DO alumnos:**
 * >
 * > ✅ 1. **Increment the System tick counter `msTicks` in 1 count.** To do so, use the function `port_system_get_millis()` and `port_system_set_millis()`.
 *
 * @warning **The variable `msTicks` must be declared volatile!** Just because it is modified by a call of an ISR, in order to avoid [*race conditions*](https://en.wikipedia.org/wiki/Race_condition). **Added to the definition** after *static*.
 *
 */
void SysTick_Handler(void)
{
    uint32_t local = port_system_get_millis();
    port_system_set_millis(local + 1);
}
/**
 * @brief This function handles Px10-Px15 global interrupts.
 * First, this function identifies the line/ pin which has raised the interruption. Then, perform the desired action. Before leaving it cleans the interrupt pending register.
 * 
 */
void EXTI15_10_IRQHandler(void)
{
    // Reactivar el contador del sistema SysTick
    port_system_systick_resume();
    // Verifica si hay una interrupción pendiente
    if (port_button_get_pending_interrupt(PORT_PARKING_BUTTON_ID))
    {
        // Retrieve the value of the GPIO of the user button
        bool button_value = port_button_get_value(PORT_PARKING_BUTTON_ID);

        // Check if the button has been pressed or released
        if (button_value)
        {
            // Button released
            port_button_set_pressed(PORT_PARKING_BUTTON_ID, false);
        }
        else
        {
            // Button pressed
            port_button_set_pressed(PORT_PARKING_BUTTON_ID, true);
        }

        // Clean the corresponding bit of the PR register
        port_button_clear_pending_interrupt(PORT_PARKING_BUTTON_ID);
    }


    // Clean the corresponding bit of the PR register
    EXTI->PR = BIT_POS_TO_MASK(PORT_PARKING_BUTTON_ID);
}


/**
 * @brief This timer controls the duration of the trigger signal of the ultrasound sensor.
 *  When the interrupt occurs it means that the time of the trigger signal has expired and must be lowered.
 * 
 */
void TIM3_IRQHandler(void){
    // Clear the interrupt flag UIF in the status register SR
    TIM3->SR &= ~TIM_SR_UIF; /*!<Update interrupt Flag              */

    // Call the function to set the flag that indicates the time of the trigger signal has expired
    port_ultrasound_set_trigger_end(PORT_REAR_PARKING_SENSOR_ID, true);
}


/**
 * @brief This timer controls the duration of the echo signal of the ultrasound sensor by means of the input capture mode.
 * The timer can interrupt in two cases:
 * 1-When the echo signal has not been received and the ARR register overflows. In this case, the echo_overflows counter is incremented.
 * 2- When the echo signal has been received. In this case, the echo_init_tick and echo_end_tick are updated.
 * 
 */
void TIM2_IRQHandler(void){

    //Reactivar el contador del sistema SysTick
    port_system_systick_resume();

    /*✅ 1. Check if the UIF flag is set. If so, this means that the ARR register has overflowed. In this case:nbsp;
    Increment the echo_overflows counter. To do this, use the functions port_ultrasound_set_echo_overflows() and port_ultrasound_get_echo_overflows() with the corresponding ultrasound ID and the incremented value.
     Remember to clear the UIF flag.*/
    // Check if the UIF flag is set
    if (TIM2->SR & TIM_SR_UIF)/* Comprobamos si la interrupcion ha sido por overflow del contador */
    {
        // Increment the echo_overflows counter
        uint32_t overflows = port_ultrasound_get_echo_overflows(PORT_REAR_PARKING_SENSOR_ID);
        port_ultrasound_set_echo_overflows(PORT_REAR_PARKING_SENSOR_ID, overflows + 1);

        // Clear the UIF flag
        TIM2->SR &= ~TIM_SR_UIF;
    }
    /* ✅ 2. Check if the CCxIF flag is set. If so, this means that the input capture event has occurred.      
     Read the value of the CCRx register to get the current tick. Reading CCRx also clears the CCxIF bit, and it is not necessary to do it later.
     If both the echo_init_tick and echo_end_tick are 0, this means that the echo signal has not started yet. In this case, update the echo_init_tick with the current tick.
     Otherwise, update the echo_end_tick with the current tick and set the echo_received flag to true.
    💡 Use the port functions port_ultrasound_get_echo_xxx and port_ultrasound_set_echo_xxx.*/

    // Check if the CCxIF flag is set
    if ((TIM2->SR & TIM_SR_CC2IF) != 0) /* Comprobamos si la interrupcion ha sido por captura de entrada */
    {
        // Read the value of the CCRx register to get the current tick
        uint32_t current_tick = TIM2->CCR2;

        // Check if the echo signal has not started yet
        if (port_ultrasound_get_echo_init_tick(PORT_REAR_PARKING_SENSOR_ID) == 0 && port_ultrasound_get_echo_end_tick(PORT_REAR_PARKING_SENSOR_ID) == 0)
        {
            // Update the echo_init_tick with the current tick
            port_ultrasound_set_echo_init_tick(PORT_REAR_PARKING_SENSOR_ID, current_tick);
        }
        else
        {
            // Update the echo_end_tick with the current tick
            port_ultrasound_set_echo_end_tick(PORT_REAR_PARKING_SENSOR_ID, current_tick);

            // Set the echo_received flag to true
            port_ultrasound_set_echo_received(PORT_REAR_PARKING_SENSOR_ID, true);
        }
    }
}

/**
 * @brief Interrupt service routine for the timer TIM5.
 * This timer controls the duration of the measurement of the ultrasound sensor. When the interrupt occurs,
 * it means that the time of the measurement has expired and a new measurement can be started.
 * 
 */
void  TIM5_IRQHandler(void)
{
//1. Clean the interrupt flag UIF in the status register SR
    TIM5->SR &= ~TIM_SR_UIF;        /*!<Update interrupt Flag              */

    //2. Set the flag trigger_ready to true
    port_ultrasound_set_trigger_ready(PORT_REAR_PARKING_SENSOR_ID, true);
}