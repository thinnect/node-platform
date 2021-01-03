/**
 * Platform I2C access management.
 *
 * Before accessing an I2C, it must first be reserved and later released
 * again so other components could use it.
 *
 * Make no assumptions about the configuration of the I2C when receiving it
 * and perform any initialization from scratch each time.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef PLATFORM_I2C_H_
#define PLATFORM_I2C_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize I2C management.
 */
void platform_i2c_management_init ();

/**
 * Get the number of I2Cs on the platform.
 * @return number of I2Cs.
 */
uint8_t platform_i2c_count ();

/**
 * Request the specified I2C.
 * @param i2c The i2c to request.
 * @param timeout_ms Request timeout (milliseconds).
 * @return true if I2C acquired and must be returned later, false if timed out.
 */
bool platform_i2c_request (void * i2c, uint32_t timeout_ms);

/**
 * Release the specified I2C.
 * @param i2c The currently held I2C to be released.
 */
void platform_i2c_release (void * i2c);

#endif//PLATFORM_I2C_H_
