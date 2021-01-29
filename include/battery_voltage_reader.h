/*
 * Battery voltage reader API.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
 */
#ifndef BATTERY_VOLTAGE_READER_H_
#define BATTERY_VOLTAGE_READER_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * Read battery voltage.
 *
 * The function will try to acquire the ADC from platform_adc and
 * returns quickly with *false* if the ADC is not available.
 *
 * @param voltage_mV - pointer for returning battery voltage in millivolts.
 * @return true if voltage read and returned.
 */
bool read_battery_voltage (uint16_t * voltage_mV);

#endif//BATTERY_VOLTAGE_READER_H_
