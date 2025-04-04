/**
 * @file stm32f4_display.c
 * @brief Portable functions to interact with the display system FSM library. All portable functions must be implemented in this file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

/* Standard C includes */

#include <stdio.h>
#include "port_display.h"
#include "port_system.h"
#include "stm32f4_system.h"
#include "stm32f4_display.h"

/* HW dependent includes */

/* Microcontroller dependent includes */

/* Defines --------------------------------------------------------------------*/

#define TIM_AS_PWM1_MASK 0x0060 /*!< Set PWM mode 1 for channel 1, reference from the book 😲 */

/* Typedefs --------------------------------------------------------------------*/

typedef struct
{
    GPIO_TypeDef *p_port_red; /*!< GPIO where the RED LED is connected */
    uint8_t pin_red; /*!< Pin number of the RED LED */
    GPIO_TypeDef *p_port_green; /*!< GPIO where the GREEN LED is connected */
    uint8_t pin_green; /*!< Pin number of the GREEN LED */
    GPIO_TypeDef *p_port_blue; /*!< GPIO where the BLUE LED is connected */
    uint8_t pin_blue; /*!< Pin number of the BLUE LED */
} stm32f4_display_hw_t; //name of the struct

/* Global variables */

static stm32f4_display_hw_t display_arr[] = {
    [PORT_REAR_PARKING_DISPLAY_ID] = {
        .p_port_red = STM32F4_REAR_PARKING_DISPLAY_RGB_R_GPIO,
        .pin_red = STM32F4_REAR_PARKING_DISPLAY_RGB_R_PIN,
        .p_port_green = STM32F4_REAR_PARKING_DISPLAY_RGB_G_GPIO,
        .pin_green = STM32F4_REAR_PARKING_DISPLAY_RGB_G_PIN,
        .p_port_blue = STM32F4_REAR_PARKING_DISPLAY_RGB_B_GPIO,
        .pin_blue = STM32F4_REAR_PARKING_DISPLAY_RGB_B_PIN
    },
};

/* Private functions -----------------------------------------------------------*/
/**
 * @brief Get the display struct with the given ID.
 * 
 * @param display_id 
 * @return stm32f4_display_hw_t* 
 * @return NULL if the display_id is not valid.
 */
stm32f4_display_hw_t *_stm32f4_display_get(uint32_t display_id)
{
    // Return the pointer to the display with the given ID. If the ID is not valid, return NULL.
    // TO-DO alumnos
    if (display_id < sizeof(display_arr) / sizeof(display_arr[0]))
    {
        return &display_arr[display_id];
    }
    else
    {
        return NULL;
    }
}

/* Public functions -----------------------------------------------------------*/

void _timer_pwm_config(uint32_t display_id)
{
    // 1. Enable the clock source of the timer

    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN; // Enable the clock for TIM4, we use APB1
    
    // 2. Disable the counter (register CR1) and enable the autoreload preaload (bit ARPE)
    TIM4->CR1 = 0x0000 ; /* Aunque es el valor por defecto */
    TIM4->CR1 &= ~TIM_CR1_CEN;  /* TIM_CR1_CEN is !<Counter enable */
    TIM4->CR1 |= TIM_CR1_ARPE;  /* TIM_CR1_ARPE is !<Auto-reload preload enable, con esto, al activar ARPE, los cambios en ARR solo funcionan cuando el temporizador se reincia */

    // 3. Reset the counter (register CNT), set the autoreload value (register ARR) and the prescaler (register PSC) for a frequency of 50 Hz.
    TIM4->CNT = 0; // Reset the counter
    TIM4->PSC = 0; // Set the prescaler to 0

    double system_clock = (double)SystemCoreClock;
    double pwm_frequency = (double)PORT_DISPLAY_FREC_MS; // 50 Hz
    double pwm_frequency_s = pwm_frequency * 1e-3; // Convert to seconds

    double psc = round((system_clock * pwm_frequency_s / 65536.0) - 1.0); // Compute an initial value for the PSC register considering the maximum value of ARR (65535.0)
    double arr = round((system_clock * pwm_frequency_s / (psc + 1.0)) - 1.0); // Compute an initial value for the ARR register
    if (arr > 65535.0) //NOTE: 65535 is like 0xFFFF
    {
        psc += 1.0; // Increment the PSC register to reduce the ARR value
        arr = round((system_clock * pwm_frequency_s / (psc + 1.0)) - 1.0);
    }
    // Set the prescaler and the auto reload register
    TIM4->PSC = (uint32_t)psc;
    TIM4->ARR = (uint32_t)arr;

    //5. Disable the output compare (register CCER) for each one of the corresponding channels. Take into account the channel number (1, 2, 3, or 4) and the channel enable bit (CCxE)
    TIM4->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E); // Disable the output compare for all channels

    // 6. Clear the P and NP bits (CCxP and CCxNP) of the output compare register (CCER) for each one of the corresponding channels.
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P | TIM_CCER_CC4P); // Clear the P and NP bits for all channels

    // 7. Set both (i) mode PWM 1, and (ii) enable preload (register CCMRx) for each one of the corresponding channels.
    //To set the mode PWM to mode 1 you must set the bits OCxM in the CCMRx register to the corresponding values according to the explanations of the manufacturer
    //To enable the preload you must set the bit OCxPE to 1 in the same register.

    TIM4->CCMR1 |= TIM_AS_PWM1_MASK; // Set PWM mode 1 for channel 1
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload for channel 1

    // 8. Generate an update event (register EGR) by setting the UG bit. This will load the values of the ARR and PSC registers into the active registers.
    TIM4->EGR |= TIM_EGR_UG;  /*!<Update Generation                         */
}

void port_display_init(uint32_t display_id)
{
    stm32f4_display_hw_t *p_display = _stm32f4_display_get(display_id);
    
    if (p_display == NULL)
    {
        return; // Invalid display ID
    }

    // 2. Call function stm32f4_system_gpio_config() with the right arguments to configure each RGB LED as in alternate mode and no pull up neither pull down connection.
    stm32f4_system_gpio_config(p_display->p_port_red, p_display->pin_red, STM32F4_GPIO_MODE_AF, STM32F4_GPIO_PUPDR_NOPULL);
    stm32f4_system_gpio_config(p_display->p_port_green, p_display->pin_green, STM32F4_GPIO_MODE_AF, STM32F4_GPIO_PUPDR_NOPULL);
    stm32f4_system_gpio_config(p_display->p_port_blue, p_display->pin_blue, STM32F4_GPIO_MODE_AF, STM32F4_GPIO_PUPDR_NOPULL);
    
    // 3. Call function stm32f4_system_gpio_config_alternate() with the right arguments to configure the alternate function of the each RGB LED.
    stm32f4_system_gpio_config_alternate(p_display->p_port_red, p_display->pin_red, STM32F4_AF2); // AF2 for TIM4_CH1
    stm32f4_system_gpio_config_alternate(p_display->p_port_green, p_display->pin_green, STM32F4_AF2); // AF2 for TIM4_CH2
    stm32f4_system_gpio_config_alternate(p_display->p_port_blue, p_display->pin_blue, STM32F4_AF2); // AF2 for TIM4_CH3

    // 4. Call function _timer_pwm_config() to configure the timer and the PWM signal of the display.
     _timer_pwm_config(display_id);

    // 5. Call function port_display_set_rgb() to set the RGB LED to off
    port_display_set_rgb(display_id, COLOR_OFF); // Set the RGB LED to off from the start
}


void port_display_set_rgb(uint32_t display_id, rgb_color_t color)
{
    stm32f4_display_hw_t *p_display = _stm32f4_display_get(display_id);
    
    if (p_display == NULL){return;}// Invalid display ID 
    // Solo configuramos si el display_id es el trasero
    if (display_id == PORT_REAR_PARKING_DISPLAY_ID)
    {
        // Desactivar el temporizador
        TIM4->CR1 &= ~TIM_CR1_CEN;

        // Extraer los valores individuales
        uint8_t r = color.r;
        uint8_t g = color.g;
        uint8_t b = color.b;

        // if all values are 0, turn off the RGB LED
        if (r == 0 && g == 0 && b == 0)
        {
            TIM4->CCER &= ~TIM_CCER_CC1E; // Desactivar canal rojo
            TIM4->CCER &= ~TIM_CCER_CC2E; // Desactivar canal verde
            TIM4->CCER &= ~TIM_CCER_CC3E; // Desactivar canal azul
            return;
        }

        // RED CHANNEL (CH1)
        // Set the duty cycle for the red channel
        if (r == 0){TIM4->CCER &= ~TIM_CCER_CC1E;}
        else
        {
            TIM4->CCR1 = r; // Duty cycle proporcional (0-255)
            TIM4->CCER |= TIM_CCER_CC1E; // Enable the output for channel 1
        }

        // GREEN CHANNEL (CH1)
        // Set the duty cycle for the red channel
        if (g == 0){TIM4->CCER &= ~TIM_CCER_CC2E;}
        else
        {
            TIM4->CCR2 = g;
            TIM4->CCER |= TIM_CCER_CC2E; // Enable the output for channel 2
        }

        // BLUE CHANNEL (CH1)
        // Set the duty cycle for the red channel
        if (b == 0){TIM4->CCER &= ~TIM_CCER_CC3E;}
        else
        {
            TIM4->CCR3 = b;
            TIM4->CCER |= TIM_CCER_CC3E; // Enable the output for channel 3
        }

        // Forzar actualización y reactivar el temporizador
        TIM4->EGR |= TIM_EGR_UG;
        TIM4->CR1 |= TIM_CR1_CEN;
    }
}



