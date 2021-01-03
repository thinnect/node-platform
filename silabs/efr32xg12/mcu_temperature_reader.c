/**
 * Measure MCU internal temperature.
 *
 * Copyright Thinnect Inc. 2019
 * @author Raido Pahtma
 * @license MIT
*/
#include "mcu_temperature_reader.h"

#include "em_device.h"
#include "em_adc.h"
#include "em_cmu.h"

#include "platform_adc.h"

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

void MCUTemperatureReader_init()
{
    ADC_Init_TypeDef ADC_Defaults = ADC_INIT_DEFAULT;
    ADC_InitSingle_TypeDef init = ADC_INITSINGLE_DEFAULT;

    init.diff = false;
    init.negSel = adcNegSelVSS;
    init.posSel = adcPosSelTEMP;
    init.reference = adcRef1V25;
    init.acqTime = adcAcqTime256;

    if(platform_adc_request(ADC0, osWaitForever))
    {
        ADC_Reset(ADC0);

        // Enable ADC clock and prescaler
        CMU_ClockEnable(cmuClock_ADC0, true);
        ADC_Defaults.prescale = ADC_PrescaleCalc(100000, 0);
        /*start with defaults */
        ADC_Init(ADC0,&ADC_Defaults);

        ADC_InitSingle(ADC0, &init);
    }
}

//------------------------------------------------------------------------------
//https://www.silabs.com/community/mcu/32-bit/knowledge-base.entry.html/2016/12/28/efr32_adc_internalt-osgP
//------------------------------------------------------------------------------

/**************************************************************************//**
 * @brief Convert ADC sample values to celsius.
 * @detail See section 25.3.4.1 in the reference manual for detail on
 *   temperature measurement and conversion.
 * @param adcSample Raw value from ADC to be converted to celsius
 * @return The temperature in degrees celsius.
 *****************************************************************************/
static float ConvertToCelsius(int32_t adcSample)
{
  uint32_t calTemp0;
  uint32_t calValue0;
  int32_t readDiff;
  float temp;

  /* Factory calibration temperature from device information page. */
  calTemp0 = ((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK)
              >> _DEVINFO_CAL_TEMP_SHIFT);

  calValue0 = ((DEVINFO->ADC0CAL3
                /* _DEVINFO_ADC0CAL3_TEMPREAD1V25_MASK is not correct in
                    current CMSIS. This is a 12-bit value, not 16-bit. */
                & 0xFFF0)
               >> _DEVINFO_ADC0CAL3_TEMPREAD1V25_SHIFT);

  if ((calTemp0 == 0xFF) || (calValue0 == 0xFFF))
  {
    /* The temperature sensor is not calibrated */
    return -100.0;
  }

  /* Vref = 1250mV
     TGRAD_ADCTH = 1.835 mV/degC (from datasheet)
  */
  readDiff = calValue0 - adcSample;
  temp     = ((float)readDiff * 1250);
  temp    /= (4096 * -1.835);

  /* Calculate offset from calibration temperature */
  temp     = (float)calTemp0 - temp;
  return temp;
}
//------------------------------------------------------------------------------

float MCUTemperatureReader_read()
{
    uint32_t sum = 0;
    uint8_t samples = 0;

    for(uint8_t i=0;i<5;i++)
    {
        unsigned int count = 0;
        ADC_Start(ADC0, adcStartSingle);

        // Wait for valid data to become available or timeout
        while(!(ADC0->STATUS & ADC_STATUS_SINGLEDV) && (count++ <= ADC_SAMPLING_TIMEOUT))
        {
            #ifdef USE_CMSIS_OS2
                osDelay(1);
            #endif
        }

        if (count <= ADC_SAMPLING_TIMEOUT)
        {
            uint16_t raw = ADC_DataSingleGet(ADC0);
            samples++;
            sum += raw;
        }
        else
        {
            err1("ADC !valid");
        }
    }

    if(samples > 0)
    {
        debug1("smp %"PRIu8" sum %"PRIu32, samples, sum);
        return ConvertToCelsius(sum/samples);
    }

    return -273.15; // Absolute zero
}

void MCUTemperatureReader_deinit() {
    CMU_ClockEnable(cmuClock_ADC0, false);
    ADC_Reset(ADC0);
    platform_adc_release(ADC0);
}
