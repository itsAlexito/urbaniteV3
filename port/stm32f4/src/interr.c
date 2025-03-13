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

void EXTI15_10_IRQHandler(void)
{
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


void TIM3_IRQHandler(void){
    // Clear the interrupt flag UIF in the status register SR
    TIM3->SR &= ~TIM_SR_UIF; /*!<Update interrupt Flag              */

    // Call the function to set the flag that indicates the time of the trigger signal has expired
    port_ultrasound_set_trigger_end(PORT_REAR_PARKING_SENSOR_ID, true);
}