/*
 * Basic counter API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Raido Pahtma
 */
#pragma once

#include <stdint.h>

/**
 * Initialize the counter, start counting up.
 */
void basic_counter_init();

/**
 * Deinitialize the counter, counting stops.
 */
void basic_counter_deinit();

/**
 * Get seconds from init, optionally milliseconds.

 * @param msec Location to store milliseconds, may be NULL.
 * @return Seconds since init.
 */
uint32_t basic_counter_get(uint16_t *msec);
