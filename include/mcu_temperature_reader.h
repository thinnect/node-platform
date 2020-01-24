/**
 * Measure MCU internal temperature.
 *
 * Copyright Thinnect Inc. 2019
 * @author Raido Pahtma
 * @license MIT
*/
#ifndef MCU_TEMPERATURE_READER_H_
#define MCU_TEMPERATURE_READER_H_

/**
 * Initialize the MCU temperature reader.
 */
void MCUTemperatureReader_init();

/**
 * Read the MCU temperature.
 * @return temperature or absolute zero on error.
 */
float MCUTemperatureReader_read();

/**
 * Deinitialize the MCU temperature reader.
 */
void MCUTemperatureReader_deinit();

#endif//MCU_TEMPERATURE_READER_H_
