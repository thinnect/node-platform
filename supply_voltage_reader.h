/*
 * MCU supply voltage reader API.
 *
 * Copyright Thinnect Inc.
 * @license MIT
 * @author Raido Pahtma
 */

#pragma once

#include <inttypes.h>

/**
 * Initialize the reader.
 */
void    SupplyVoltageReader_init();

/**
 * @return Supply voltage in mV.
 */
int16_t SupplyVoltageReader_read();

/**
 * Deinitialize the reader.
 */
void    SupplyVoltageReader_deinit();
