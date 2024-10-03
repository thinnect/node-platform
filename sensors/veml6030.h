/**
 * VEML6030 light sensor I2C driver.
 *
 * Copyright Thinnect Inc. 2023
 * @license MIT
 */
#ifndef VEML6030_H_
#define VEML6030_H_

#include <stdint.h>

#define VEML6030_ADDR_0x10 0x10
#define VEML6030_ADDR_0x48 0x48

/**
 * @brief Initialize the VEML6030 light sensor.
 *
 * @param i2c Pointer to platform_i2c instance to use.
 * @param addr 7-bit address of the sensor (0x10 or 0x48).
 */
void veml6030_init (void * i2c, uint8_t addr);

/**
 * @brief Read the light sensor.
 *
 * @param integration_time_ms Integration time to use when sampling
 *                            the sensor 25-800ms, power of 2 times 25ms.
 * @return Lux value (>= 0) or negative for errors.
 */
int32_t veml6030_read (uint16_t integration_time_ms);

#endif//VEML6030_H_
