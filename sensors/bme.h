/*
 * Really basic wrapper around the Bosch BME280 driver.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Raido Pahtma
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool bme280_read(int32_t *temperature, uint32_t *pressure, uint32_t *humidity);
