/**
 * @file port_ultrasound.h
 * @brief Header for the portable functions to interact with the HW of the ultrasound sensors. The functions must be implemented in the platform-specific code.
 * @author alumno1
 * @author alumno2
 * @date fecha
 */
#ifndef PORT_ULTRASOUND_H_
#define PORT_ULTRASOUND_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Standard C includes */

/* Defines and enums ----------------------------------------------------------*/

#define PORT_REAR_PARKING_SENSOR_ID 0
#define PORT_PARKING_SENSOR_TRIGGER_UP_US 10
#define PORT_PARKING_SENSOR_TIMEOUT_MS 100
#define SPEED_OF_SOUND 343 // m/s

/* Function prototypes and explanation -------------------------------------------------*/

void port_ultrasound_init(uint32_t ultrasound_id);

void port_ultrasound_start_measurement(uint32_t ultrasound_id);

void port_ultrasound_stop_trigger_timer(uint32_t ultrasound_id);

void port_ultrasound_stop_echo_timer(uint32_t ultrasound_id);

void port_ultrasound_start_new_measurement_timer(void);

void port_ultrasound_stop_new_measurement_timer(void);

void port_ultrasound_reset_echo_ticks(uint32_t ultrasound_id);

void port_ultrasound_stop_ultrasound(uint32_t ultrasound_id);

bool port_ultrasound_get_trigger_ready (uint32_t ultrasound_id);

void port_ultrasound_set_trigger_ready (uint32_t ultrasound_id, bool trigger_ready);

bool port_ultrasound_get_trigger_end (uint32_t ultrasound_id);

void port_ultrasound_set_trigger_end (uint32_t ultrasound_id, bool trigger_end);

uint32_t port_ultrasound_get_echo_init_tick (uint32_t ultrasound_id);

void port_ultrasound_set_echo_init_tick (uint32_t ultrasound_id, uint32_t echo_init_tick);

uint32_t port_ultrasound_get_echo_end_tick (uint32_t ultrasound_id);

void port_ultrasound_set_echo_end_tick (uint32_t ultrasound_id, uint32_t echo_end_tick);

bool port_ultrasound_get_echo_received (uint32_t ultrasound_id);

void port_ultrasound_set_echo_received (uint32_t ultrasound_id, bool echo_received);

uint32_t port_ultrasound_get_echo_overflows (uint32_t ultrasound_id);

void port_ultrasound_set_echo_overflows (uint32_t ultrasound_id, uint32_t echo_overflows);



#endif /* PORT_ULTRASOUND_H_ */
