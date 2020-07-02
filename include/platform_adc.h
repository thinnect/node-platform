/**
 * Platform ADC access management.
 *
 * Before accessing an ADC, it must first be reserved and later released
 * again so other components could use it.
 *
 * Make no assumptions about the configuration of the ADC when receiving it
 * and perform any initialization from scratch each time.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef PLATFORM_ADC_H_
#define PLATFORM_ADC_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Initialize ADC management.
 */
void platform_adc_management_init ();

/**
 * Get the number of ADCs on the platform.
 * @return number of ADCs.
 */
uint8_t platform_adc_count ();

/**
 * Request the specified ADC.
 * @param adc The adc to request.
 * @param timeout_ms Request timeout (milliseconds).
 * @return true if ADC acquired and must be returned later, false if timed out.
 */
bool platform_adc_request (void * adc, uint32_t timeout_ms);

/**
 * Release the specified ADC.
 * @param adc The currently held ADC to be released.
 */
void platform_adc_release (void * adc);

#endif//PLATFORM_ADC_H_
