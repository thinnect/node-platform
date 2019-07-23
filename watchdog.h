/*
 * MCU watchdog API.
 *
 * The watchdog, once enabled, will require periodic feeding, otherwise it
 * will reset the MCU. Optionally a reminder may be configured to notify before
 * an actual reset occurs.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifndef WATCHDOG_PERIOD
#define WATCHDOG_PERIOD 4000
#endif

/**
 * Feed the watchdog.
 */
void watchdog_feed();

/**
 * Enable the watchdog with the specified period.
 *
 * @param timeout_ms Watchdog timeout in milliseconds.
 *
 * @return true if the warning was enabled, false otherwise
 */
bool watchdog_enable(uint32_t timeout_ms);

/**
 * Check the watchdog period and state.
 *
 * @return watchdog period or UINT32_MAX if disabled.
 */
uint32_t watchdog_period();

/**
 * Disable the watchdog.
 */
void watchdog_disable();

/**
 * Watchdog warning callback.
 *
 * The user can feed the watchdog from the callback by returning true.
 *
 * @param remaining_ms Time remaining until watchdog resets the MCU
 *                     or UINT32_MAX if only warning enabled.
 * @return bool to feed the watchdog.
 */
typedef bool watchdog_warning_f(uint32_t remaining_ms);

/**
 * Enable a watchdog warning with the specified period. Warning functionality
 * may not be supported on all platforms, in which case false will be returned.
 *
 * @param timeout_ms Watchdog warning timeout in milliseconds. Must be less than
 *                   or equal to the actual watchdog timeout.
 * @param callback Function to call when the warning timeout is exceeded.
 * @return true if the warning was enabled
 */
bool watchdog_enable_warning(uint32_t timeout_ms, watchdog_warning_f* callback);

/**
 * Check the watchdog warning period and state.
 *
 * @return watchdog period or UINT32_MAX if disabled.
 */
uint32_t watchdog_warning_period();

/**
 * Disable the watchdog warning.
 */
void watchdog_disable_warning();
