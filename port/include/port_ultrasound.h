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

#define PORT_REAR_PARKING_SENSOR_ID 0 /*!< ID of the rear parking sensor */
#define PORT_PARKING_SENSOR_TRIGGER_UP_US 10 /*!< Duration of the trigger signal in microseconds */
#define PORT_PARKING_SENSOR_TIMEOUT_MS 100 /*!< Timeout for the ultrasound sensor in milliseconds */
#define SPEED_OF_SOUND 343 /*!< Speed of sound in air in m/s */

/* Function prototypes and explanation -------------------------------------------------*/

/**
 * @brief Configure the HW specifications of a given ultrasound sensor.
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_init(uint32_t ultrasound_id);


/**
 * @brief this function starts the measurement of the ultrasound sensor.
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_start_measurement(uint32_t ultrasound_id);

/**
 * @brief Stop the timer that controls the trigger signal.
 *This function stops the timer that controls the trigger signal because the time to trigger the ultrasound sensor has finished.
 *It also sets the trigger signal to low.
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_stop_trigger_timer(uint32_t ultrasound_id);

/**
 * @brief  Stop the timer that controls the trigger signal.
 *This function stops the timer that controls the trigger signal because the time to trigger the ultrasound sensor has finished.
 *It also sets the trigger signal to low.
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_stop_echo_timer(uint32_t ultrasound_id);

/**
 * @brief this function starts the measurement of the ultrasound sensor.
 * 
 */
void port_ultrasound_start_new_measurement_timer(void);


/**
 * @brief Stop the timer that controls the new measurement
 * this function stops the timer that controls the new measurement 
 * 
 */
void port_ultrasound_stop_new_measurement_timer(void);

/**
 * @brief This function resets the time ticks of the echo signal once the distance has been calculated.
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_reset_echo_ticks(uint32_t ultrasound_id);

/**
 * @brief Stop the timers of the ultrasound sensor and reset the echo ticks
 * 
 * @param ultrasound_id 
 */
void port_ultrasound_stop_ultrasound(uint32_t ultrasound_id);

/**
 * @brief Get the readiness of the trigger signal.
 * This function returns the status of readiness the trigger signal. If it is true, the ultrasound sensor is ready to start a new measurement.
 * 
 * @param ultrasound_id 
 * @return true 
 * @return false 
 */
bool port_ultrasound_get_trigger_ready (uint32_t ultrasound_id);


/**
 * @brief Set the readiness of the trigger signal.
 * This function sets the status of readiness of the trigger signal.
 * If it is true, the ultrasound sensor will ready to start a new measurement. 
 * 
 * @param ultrasound_id 
 * @param trigger_ready 
 */
void port_ultrasound_set_trigger_ready (uint32_t ultrasound_id, bool trigger_ready);


/**
 * @brief Get the status of the trigger signal.
 * This function returns the status of the trigger signal. It will be true if the time to trigger the ultrasound sensor has finished.
 * 
 * @param ultrasound_id 
 * @return true if the trigger signal has ended
 * @return false if the trigger signal has not ended
 */
bool port_ultrasound_get_trigger_end (uint32_t ultrasound_id);


/**
 * @brief Set the status of the trigger signal.
 * 
 * @param ultrasound_id 
 * @param trigger_end 
 */
void port_ultrasound_set_trigger_end (uint32_t ultrasound_id, bool trigger_end);


/**
 * @brief Get the time tick when the init of echo signal was received.
 * 
 * @param ultrasound_id 
 * @return uint32_t 
 */
uint32_t port_ultrasound_get_echo_init_tick (uint32_t ultrasound_id);



/**
 * @brief This function sets the time tick when the init of echo signal was received.
 *  It is called by the ISR of the input capture of the echo signal.
 * 
 * @param ultrasound_id 
 * @param echo_init_tick 
 */
void port_ultrasound_set_echo_init_tick (uint32_t ultrasound_id, uint32_t echo_init_tick);

/**
 * @brief Get the time tick when the end of echo signal was received.
 * 
 * @param ultrasound_id 
 * @return uint32_t 
 */
uint32_t port_ultrasound_get_echo_end_tick (uint32_t ultrasound_id);

/**
 * @brief This function sets the time tick when the end of echo signal was received. 
 * It is called by the ISR of the input capture of the echo signal.
 * 
 * @param ultrasound_id 
 * @param echo_end_tick 
 */
void port_ultrasound_set_echo_end_tick (uint32_t ultrasound_id, uint32_t echo_end_tick);

/**
 * @brief This function returns the status of the echo signal. 
 * It will be true if the echo signal has been received (both the init and end ticks).
 * 
 * @param ultrasound_id 
 * @return true 
 * @return false 
 */
bool port_ultrasound_get_echo_received (uint32_t ultrasound_id);

/**
 * @brief This function sets the status of the echo signal.
 *  The ISR of the input capture of the echo signal calls this function to set the status of the echo signal
 * when both the init and end ticks have been received.
 * 
 * @param ultrasound_id 
 * @param echo_received 
 */
void port_ultrasound_set_echo_received (uint32_t ultrasound_id, bool echo_received);

/**
 * @brief This function returns the number of overflows of the echo signal timer.
 *  It is used to calculate the real time elapsed in the echo signal.
 * 
 * @param ultrasound_id 
 * @return uint32_t 
 */
uint32_t port_ultrasound_get_echo_overflows (uint32_t ultrasound_id);

/**
 * @brief This function sets the number of overflows of the echo signal timer. 
 * It is called by the ISR of the input capture of the echo signal
 *  when an overflow occurs to increment the number of overflows.
 * 
 * @param ultrasound_id 
 * @param echo_overflows 
 */
void port_ultrasound_set_echo_overflows (uint32_t ultrasound_id, uint32_t echo_overflows);



#endif /* PORT_ULTRASOUND_H_ */
