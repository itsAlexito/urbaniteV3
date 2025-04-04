/**
 * @file port_display.h
 * @brief Header for the portable functions to interact with the HW of the display system. The functions must be implemented in the platform-specific code.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */
#ifndef PORT_DISPLAY_SYSTEM_H_
#define PORT_DISPLAY_SYSTEM_H_

/* Includes ------------------------------------------------------------------*/
/* Standard C includes */
#include <stdint.h>

/* Typedefs --------------------------------------------------------------------*/

/* Typedefs --------------------------------------------------------------------*/
typedef struct {
	uint8_t r; /*!<Red color value */
	uint8_t g; /*!<Green color value */
	uint8_t b; /*!<Blue color value */
} rgb_color_t;

/* Defines and enums ----------------------------------------------------------*/
/* Defines */
#define PORT_REAR_PARKING_DISPLAY_ID   0
#define PORT_DISPLAY_RGB_MAX_VALUE 255
#define COLOR_RED       (rgb_color_t){255, 0, 0}       /*!<Rojo (danger) */
#define COLOR_YELLOW    (rgb_color_t){255, 255, 0}     /*!< Amarillo (warning) */
#define COLOR_GREEN     (rgb_color_t){0, 255, 0}       /*!< Verde (no problem) */
#define COLOR_TURQUOISE (rgb_color_t){64, 224, 208}    /*!< Turquesa (Info) */
#define COLOR_BLUE      (rgb_color_t){0, 0, 255}       /*!< Azul (Info) */
#define COLOR_OFF       (rgb_color_t){0, 0, 0}         /*!< Apagado (off) */
#define PORT_DISPLAY_FREC_MS 20 /*!< Frequency of the display in ms  (50 HZ)*/

/* Function prototypes and explanation -------------------------------------------------*/

/**
 * @brief Configure the HW specifications of a given display.
 * Assuming we are using an STM32F4-based platform, this function must call the following functions
 * 
 * @param display_id 
 */
void port_display_init(uint32_t display_id);

/**
 * @brief Set the Capture/Compare register values for each channel of the RGB LED given a color.
 * This function disables the timer associated to the RGB LEDs, sets the Capture/Compare register values for each channel of the RGB LED, and enables the timer.
 * 
 * @image html RGBFlowChart.png
 * @warning This function is valid for any given RGB LED, however, each RGB LED has its own timer. 
 * @param display_id 
 * @param color 
 */
void port_display_set_rgb(uint32_t display_id ,rgb_color_t color);



#endif /* PORT_DISPLAY_SYSTEM_H_ */