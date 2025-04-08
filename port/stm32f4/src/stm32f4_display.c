/**
 * @file stm32f4_display.c
 * @brief Portable functions to interact with the display system FSM library. All portable functions must be implemented in this file.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */

/* Standard C includes */

#include <stdio.h>
#include <math.h>



/* HW dependent includes */
#include "port_display.h"
#include "port_system.h"
#include "stm32f4_system.h"
#include "stm32f4_display.h"
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
    // Paso 1: Activar el reloj del timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Paso 2: Deshabilitar el contador y habilitar ARPE (auto-reload preload)
    TIM4->CR1 = 0x0000;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    TIM4->CR1 |= TIM_CR1_ARPE;

    // Paso 3: Configurar ARR, PSC y CNT para 50Hz (TPWM = 20ms)
    TIM4->CNT = 0;

    double freq_ms = PORT_DISPLAY_FREC_MS; // 20 ms
    double t_pwm_s = freq_ms / 1000.0;
    double f_clk = SystemCoreClock;

    double psc_d = round((f_clk * t_pwm_s) / 65536.0 - 1.0);
    double arr_d = round((f_clk * t_pwm_s) / (psc_d + 1.0) - 1.0);
    if (arr_d > 65535.0)
    {
        psc_d += 1.0;
        arr_d = round((f_clk * t_pwm_s) / (psc_d + 1.0) - 1.0);
    }

    TIM4->PSC = (uint32_t)psc_d;
    TIM4->ARR = (uint32_t)arr_d;

    // Paso 8: Generar evento de actualización para cargar ARR y PSC
    TIM4->EGR = TIM_EGR_UG;

    // Paso 5: Deshabilitar salida de canales
    TIM4->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);

    // Paso 6: Limpiar bits de polaridad (P y NP)
    TIM4->CCER &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);

    // Paso 7: Configurar modo PWM1 y activar preload para cada canal
    // CH1 (Rojo)
    TIM4->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM Mode 1
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;

    // CH3 (verde)
    TIM4->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
    TIM4->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM Mode 1
    TIM4->CCMR2 |= TIM_CCMR2_OC3PE;

    // CH4  (azul)
    TIM4->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
    TIM4->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // PWM Mode 1
    TIM4->CCMR2 |= TIM_CCMR2_OC4PE;

    // ¡No activar el timer ni CCER aquí! Se hace en port_display_set_rgb
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
        if (r == 0)
        {TIM4->CCER &= ~TIM_CCER_CC1E;
        }
        else
        {
            TIM4->CCR1 =  ((uint32_t)r * (TIM4->ARR + 1)) / PORT_DISPLAY_RGB_MAX_VALUE;
            TIM4->CCER |= TIM_CCER_CC1E; // Enable the output for channel 1
        }

        // GREEN CHANNEL (CH3)
        // Set the duty cycle for the red channel
        if (g == 0){TIM4->CCER &= ~TIM_CCER_CC3E;}
        else
        {
            TIM4->CCR3 = ((uint32_t)g * (TIM4->ARR + 1)) / PORT_DISPLAY_RGB_MAX_VALUE;
            TIM4->CCER |= TIM_CCER_CC3E; // Enable the output for channel 3
        }

        // BLUE CHANNEL (CH4)
        // Set the duty cycle for the blue channel
        if (b == 0)
        {TIM4->CCER &= ~TIM_CCER_CC4E;
        }
        else
        {
            TIM4->CCR4 = ((uint32_t)b * (TIM4->ARR + 1)) / PORT_DISPLAY_RGB_MAX_VALUE;
            TIM4->CCER |= TIM_CCER_CC4E; // Enable the output for channel 3
        }

        // Forzar actualización y reactivar el temporizador
        TIM4->EGR |= TIM_EGR_UG;
        TIM4->CR1 |= TIM_CR1_CEN;
    }
}



