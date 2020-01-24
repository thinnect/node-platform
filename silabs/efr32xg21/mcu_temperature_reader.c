/**
 * Measure MCU internal temperature.
 *
 * Copyright Thinnect Inc. 2020
 * @author Raido Pahtma
 * @license MIT
*/
#include "mcu_temperature_reader.h"

#include "em_device.h"
#include "em_adc.h"
#include "em_cmu.h"

#include <stdio.h>

#include "loglevels.h"
#define __MODUUL__ "mcut"
#define __LOG_LEVEL__ (LOG_LEVEL_mcut & BASE_LOG_LEVEL)
#include "log.h"

#ifdef USE_CMSIS_OS2
#include "cmsis_os2.h"
#define ADC_SAMPLING_TIMEOUT 100
#else
#define ADC_SAMPLING_TIMEOUT 1000000
#endif//USE_CMSIS_OS2

#warning "MCU temperature reader not implemented, reads always return 0!"

void MCUTemperatureReader_init ()
{
    // TODO
}

float MCUTemperatureReader_read ()
{
    return 0; // TODO
}

void MCUTemperatureReader_deinit ()
{

}
