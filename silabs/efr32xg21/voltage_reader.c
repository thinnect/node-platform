/**
 * Semi-generic battery voltage reader for EFR32xG21.
 *
 * Copyright Thinnect Inc. 2019
 * @param license <PROPRIETARY>
 */
#include "platform_io.h"
#include "platform_adc.h"
#include "em_gpio.h"

#include <em_iadc.h>
#include <em_cmu.h>
#include <inttypes.h>

#include "cmsis_os2.h"

#include "loglevels.h"
#define __MODUUL__ "vltrdr"
#define __LOG_LEVEL__ (LOG_LEVEL_voltage_reader & BASE_LOG_LEVEL)
#include "log.h"

// Set HFRCOEM23 to lowest frequency (1MHz)
#define HFRCOEM23_FREQ          cmuHFRCOEM23Freq_1M0Hz
// Set CLK_ADC to 10kHz (this corresponds to a sample rate of 1ksps)
#define CLK_SRC_ADC_FREQ        1000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ            10000   // CLK_ADC

// ----------------------------------------------------------------------------
//  Init IADC system
//  @param param port - GPIO port used for voltage measurement, e.g.: gpioPortB
//  @param pin - GPIO port used for voltage measurement, e.g.: 1
//  @param pos_inp_port_pin - IADC port & pin, e.g.: iadcPosInputPortBPin1
//  @param ref_vcc_voltage - reference voltage, e.g.: 5000
//  @param ref_gnd_voltage - reference voltage, e.g.: 5000
//  @param iadc_ref - IADC Reference, e.g.: iadcCfgReferenceInt1V2
//  @param ref_voltage - reference voltage, e.g.: 2420
//  @param analog_gain - IADC Analog Gain, e.g.: iadcCfgAnalogGain0P5x
//  @param *voltage_mV - pointer where to store voltage reading
//  @return true if voltage reading was successful
// ----------------------------------------------------------------------------
bool voltage_reader (GPIO_Port_TypeDef port, 
                     unsigned int pin,
                     IADC_PosInput_t pos_inp_port_pin,
                     uint32_t ref_vcc_voltage,
                     uint32_t ref_gnd_voltage,
                     IADC_CfgReference_t iadc_ref,
                     uint32_t ref_voltage,
                     IADC_CfgAnalogGain_t analog_gain,
                     uint16_t* voltage_mV)
{
    IADC_Init_t init = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
    IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

    // Reset IADC to reset configuration in case it has been modified
    IADC_reset(IADC0);

    // Configure ADC input
    CMU_ClockEnable(cmuClock_GPIO, true);
    GPIO_PinModeSet(port, pin, gpioModeDisabled, 0);

    // Wait a bit for things to stabilize
    osDelay(10);

    // Set clock frequency to defined value
    CMU_HFRCOEM23BandSet(HFRCOEM23_FREQ);

    // Configure IADC clock source for use while in EM2
    // Note that HFRCOEM23 is the lowest frequency source available for the IADC
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_HFRCOEM23);
    CMU_ClockEnable(cmuClock_IADCCLK, true);

    // Modify init structs and initialize
    init.warmup = iadcWarmupNormal;

    // Set the HFSCLK prescale value here
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

    // Configuration 0 is used by both scan and single conversions by default
    // Use unbuffered AVDD as reference
    initAllConfigs.configs[0].reference = iadc_ref;
    initAllConfigs.configs[0].analogGain = analog_gain;

    // Divides CLK_SRC_ADC to set the CLK_ADC frequency for desired sample rate
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                       CLK_ADC_FREQ,
                                                                       0,
                                                                       iadcCfgModeNormal,
                                                                       init.srcClkPrescale);
    // Single initialization
    initSingle.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID1;
    initSingle.showId = true;
    initSingle.triggerAction = iadcTriggerActionOnce;
    initSingleInput.posInput = pos_inp_port_pin;
    initSingleInput.negInput = iadcNegInputGnd;

    // Initialize IADC
    IADC_init(IADC0, &init, &initAllConfigs);

    // Initialize Scan
    IADC_initSingle(IADC0, &initSingle, &initSingleInput);

    // Allocate the analog bus for ADC0 inputs
    GPIO->CDBUSALLOC |= GPIO_CDBUSALLOC_CDEVEN0_ADC0;

    //debug1("pop %lX", IADC0->SINGLE);

    uint32_t sum = 0;
    uint8_t num = 0;

    for (uint8_t i = 0; i < 5; ++i)
    {
        IADC_command(IADC0, iadcCmdStartSingle);

        uint32_t status = IADC_getStatus(IADC0);
        debug1("start st %X", (unsigned int)status);

        uint32_t count = 0;
        while(!(status & IADC_STATUS_SINGLEFIFODV) && (count++ <= 1000000))
        {
            count++;
            status = IADC_getStatus(IADC0);
        }

        uint32_t ints = IADC_getInt(IADC0);
        debug1("end st:%"PRIX32" c:%"PRIu32" IF:%"PRIX32, status, count, ints);

        if((count > 1000000)||(IADC_IF_POLARITYERR & ints))
        {
            err1("c:%"PRIu32" IF:%"PRIX32, count, ints);
        }
        else
        {
            IADC_Result_t sample = IADC_pullSingleFifoResult(IADC0);
            debug1("smpl %lu %u", sample.data, sample.id);
            sum += sample.data;
            num++;
        }
    }

    // Deallocate the analog bus
    GPIO->CDBUSALLOC &= ~(GPIO_CDBUSALLOC_CDEVEN0_ADC0);

    IADC_reset(IADC0);
    CMU_ClockEnable(cmuClock_IADCCLK, false);

    if (num > 0)
    {
        uint32_t avg = sum / num;
        uint32_t fscale = avg * (ref_vcc_voltage + ref_gnd_voltage) / ref_gnd_voltage;
        *voltage_mV = ref_voltage * fscale / (1 << 12); // ADC is 12 bits
        return true;
    }
    return false;
}
