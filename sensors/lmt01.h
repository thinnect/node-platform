/**
 * Driver for LMT01 from Texas Instruments
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#ifndef _LMT01_H_
#define _LMT01_H_

#include <stdint.h>

/**
 * Initializes LMT01 driver
 */
void lmt01_init();

/**
 * Read temperature from LMT01
 *
 * @return Returns temperature in C * 10
 */
int32_t lmt01_read_temperature();

#endif

