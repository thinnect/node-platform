/**
 * Measure AVDD, pretend it is the supply voltage.
 *
 * Solution based on:
 * https://www.silabs.com/community/wireless/bluetooth/knowledge-base.entry.html/2017/06/02/reporting_batteryvo-QRNB
 *
 * Copyright Thinnect Inc. 2019
 * @author Tarmo Kuuse, Konstantin Bilozor
 * @license MIT
 *
*/
#include "supply_voltage_reader.h"

#include "em_device.h"
#include "em_adc.h"
#include "em_cmu.h"

#include <stdio.h>

#define VINATT(ATT_FACTOR) ATT_FACTOR << _ADC_SINGLECTRLX_VINATT_SHIFT
#define VREFATT(ATT_FACTOR)ATT_FACTOR << _ADC_SINGLECTRLX_VREFATT_SHIFT
#define microvoltsPerStep 1221// 1322

void SupplyVoltageReader_init()
{
    ADC_Init_TypeDef ADC_Defaults = ADC_INIT_DEFAULT;

    ADC_InitSingle_TypeDef init = ADC_INITSINGLE_DEFAULT;

    init.negSel = adcNegSelVSS;
    init.posSel = adcPosSelAVDD;
    init.reference = adcRef5V ;

    ADC_Reset(ADC0);

    // Enable ADC clock and prescaler
    CMU_ClockEnable(cmuClock_ADC0, true);
    ADC_Defaults.prescale = ADC_PrescaleCalc(100000, 0);
    /*start with defaults */
    ADC_Init(ADC0,&ADC_Defaults);

    ADC_InitSingle(ADC0, &init);
    //ADC0->SINGLECTRLX = VINATT(12) | VREFATT(6);
}

/*
check to see if conversion is complete
*/
static bool ADC_SingleDataValid()
{
    return ADC0->STATUS & ADC_STATUS_SINGLEDV;
}

/*
read ADC value

VFS = 2?VREF?VREFATTF/VINATTF, where
VREF is selected in the VREFSEL bitfield, and
VREFATTF (VREF attenuation factor) = (VREFATT+6)/24 when VREFATT is less than 13, and (VREFATT-3)/12 when VREFATT is
greater than or equal to 13, and
VINATTF (VIN attenuation factor) = VINATT/12, illegal settings: 0,1,2
VREFATTF = (6+6)/24 = 0.5
VINATTF = 12/12
VFS = 2*5.0*0.5 = 5.0
1.221 mV/step
*/
int16_t SupplyVoltageReader_read()
{
    unsigned int raw=0;
    unsigned int supplyVoltagemV=0;
    unsigned int count = 0;

    ADC_Start(ADC0, adcStartSingle);

    /* wait for conversion to complete.*/
    while((ADC_IntGet(ADC0) & ADC_IF_SINGLE) != ADC_IF_SINGLE && count++ <= 1000000)
    {}

    if (count >= 1000000)
    {
        // printf("ADC setup failed st=x%"PRIx32" f=x%"PRIx32"\n", (uint32_t) ADC_IntGet(ADC0), (uint32_t) ADC_IF_SINGLE);
    }

    count = 0;
    while (!ADC_SingleDataValid() && count++ <= 1000000) {}

    if (count <= 1000000)
    {
        raw =  ADC_DataSingleGet(ADC0);

        supplyVoltagemV = raw*microvoltsPerStep/1000UL;
        return supplyVoltagemV;
    } else {
        // printf("ADC sample failed st=x%"PRIx32" v=%"PRIu32"\n", (uint32_t) ADC0->STATUS, (uint32_t) ADC_DataSingleGet(ADC0));
    }

    return -1;
}

void SupplyVoltageReader_deinit() {
    CMU_ClockEnable(cmuClock_ADC0, false);
    ADC_Reset(ADC0);
}
