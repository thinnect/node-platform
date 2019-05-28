/**
  @file supply_voltage_reader.c
  @brief Measure AVDD, pretend it is the supply voltage.
*/
#include "supply_voltage_reader.h"

#include "em_device.h"
#include "em_iadc.h"
#include "em_gpio.h"
#include "em_cmu.h"

#include <stdio.h>

#include "loglevels.h"
#define __MODUUL__ "svr"
#define __LOG_LEVEL__ (LOG_LEVEL_supply_voltage_reader & BASE_LOG_LEVEL)
#include "log.h"

// Set HFRCOEM23 to lowest frequency (1MHz)
#define HFRCOEM23_FREQ          cmuHFRCOEM23Freq_1M0Hz

// Set CLK_ADC to 10kHz (this corresponds to a sample rate of 1ksps)
#define CLK_SRC_ADC_FREQ        1000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ            10000   // CLK_ADC

void SupplyVoltageReader_init() {
    // Set clock frequency to defined value
    CMU_HFRCOEM23BandSet(HFRCOEM23_FREQ);

    IADC_Init_t init = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

    // Reset IADC to reset configuration in case it has been modified
    IADC_reset(IADC0);

    // Configure IADC clock source for use while in EM2
    // Note that HFRCOEM23 is the lowest frequency source available for the IADC
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_HFRCOEM23);
    //CMU_ClockEnable(cmuClock_IADCCLK, true);

    // Modify init structs and initialize
    init.warmup = iadcWarmupKeepWarm;

    // Set the HFSCLK prescale value here
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

    // Configuration 0 is used by both scan and single conversions by default
    // Use unbuffered AVDD as reference
    initAllConfigs.configs[0].reference = iadcCfgReferenceInt1V2; //iadcCfgReferenceInt1V2;

    // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                    CLK_ADC_FREQ,
                                                                    0,
                                                                    iadcCfgModeNormal,
                                                                    init.srcClkPrescale);

    // Single initialization
    initSingle.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID1;

    // Set conversions to run continuously
    //initSingle.triggerAction = iadcTriggerActionOnce;

    // Sample AVDD / 4
    initSingleInput.posInput = iadcPosInputAvdd;
    //When selecting SUPPLY for PORTPOS and GND for PORTNEG, PINNEG should be
    //configured for an odd number (1, 3, 5...) to avoid a polarity error.
    initSingleInput.negInput = iadcNegInputGnd + 1; // So +1


    // Initialize IADC
    IADC_init(IADC0, &init, &initAllConfigs);

    // Initialize Scan
    IADC_initSingle(IADC0, &initSingle, &initSingleInput);

    debug1("pop %lX", IADC0->SINGLE);
}

int16_t SupplyVoltageReader_read() {
    uint32_t avg = 0;
    uint32_t num = 0;
    for(uint8_t i=0; i<3;i++) {
        uint32_t count = 0;
        uint32_t status;

        IADC_command(IADC0, iadcCmdStartSingle);

        status = IADC0->STATUS;
        debug1("start %lX", status);
        while(!(status & _IADC_STATUS_SINGLEFIFODV_MASK) && (count++ <= 1000000)) {
            count++;
            status = IADC0->STATUS;
            //debug1("converting %lX", status);
        }

        if(count > 1000000) {
            err1("failed %"PRIu32"", count);
            return 0;
        } else {
            IADC_Result_t sample = IADC_pullSingleFifoResult(IADC0);
            debug1("smpl %lu %u", sample.data, sample.id);
            avg += sample.data;
            num++;
        }
    }
    // Calculate input voltage:
    //  AVDD/4 vs 12-bit 1.21 and then to millivolts
    return (int16_t)(4*1000*1.21*(avg/num)/0xFFF);
}

void SupplyVoltageReader_deinit() {
    IADC_reset(IADC0);
    // TODO Stop Clock?
}
