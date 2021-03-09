/**
 * Semi-generic battery voltage reader for EFR32xG21.
 *
 * Copyright Thinnect Inc. 2019
 * @license <PROPRIETARY>
 */
#include "platform_io.h"
#include "platform_adc.h"

#include <em_iadc.h>
#include <em_cmu.h>
#include <inttypes.h>

#include "cmsis_os2.h"

#include "loglevels.h"
#define __MODUUL__ "rbat"
#define __LOG_LEVEL__ (LOG_LEVEL_battery_voltage_reader & BASE_LOG_LEVEL)
#include "log.h"

#ifndef PLATFORM_BATV_ENABLE_PORT
#error "PLATFORM_BATV_ENABLE_PORT undefined (example: gpioPortC)"
#endif//PLATFORM_BATV_ENABLE_PORT
#ifndef PLATFORM_BATV_ENABLE_PIN
#error "PLATFORM_BATV_ENABLE_PIN undefined (example: 5)"
#endif// PLATFORM_BATV_ENABLE_PIN
#ifndef PLATFORM_BATV_ENABLE_LEVEL
#error "PLATFORM_BATV_ENABLE_LEVEL undefined (0 or 1)"
#endif// PLATFORM_BATV_ENABLE_LEVEL
#ifndef PLATFORM_BATV_DISABLE_LEVEL
#error "PLATFORM_BATV_DISABLE_LEVEL undefined (0, 1 or 3 for hi-Z)"
#endif// PLATFORM_BATV_DISABLE_LEVEL
#ifndef PLATFORM_BATV_INPUT_PORT
#error "PLATFORM_BATV_INPUT_PORT undefined (example: gpioPortC)"
#endif//PLATFORM_BATV_INPUT_PORT
#ifndef PLATFORM_BATV_INPUT_PIN
#error "PLATFORM_BATV_INPUT_PIN undefined (example: 4)"
#endif//PLATFORM_BATV_INPUT_PIN
#ifndef PLATFORM_BATV_INPUT_IADC_PORTPIN
#error "PLATFORM_BATV_INPUT_IADC_PORTPIN undefined (example: iadcPosInputPortCPin4)"
#endif//PLATFORM_BATV_INPUT_IADC_PORTPIN
#ifndef PLATFORM_BATV_INPUT_IADC_BUS
#error "PLATFORM_BATV_INPUT_IADC_BUS undefined (example: CDBUSALLOC)"
#endif//PLATFORM_BATV_INPUT_IADC_BUS
#ifndef PLATFORM_BATV_INPUT_IADC_BUSALLOC
#error "PLATFORM_BATV_INPUT_IADC_BUSALLOC undefined (example: GPIO_CDBUSALLOC_CDEVEN0_ADC0)"
#endif//PLATFORM_BATV_INPUT_IADC_BUSALLOC
#ifndef PLATFORM_BATV_R_VCC
#error "PLATFORM_BATV_R_VCC undefined (example: 4700)"
#endif//PLATFORM_BATV_R_VCC
#ifndef PLATFORM_BATV_R_GND
#error "PLATFORM_BATV_R_GND undefined (example: 4700)"
#endif//PLATFORM_BATV_R_GND
#ifndef PLATFORM_BATV_REFERENCE_IADC
#error "PLATFORM_BATV_REFERENCE_IADC undefined (example: iadcCfgReferenceInt1V2)"
#endif//PLATFORM_BATV_REFERENCE_IADC
#ifndef PLATFORM_BATV_REFERENCE_VOLTAGE
#error "PLATFORM_BATV_REFERENCE_VOLTAGE undefined (example: 2420)"
#endif//PLATFORM_BATV_REFERENCE_VOLTAGE
#ifndef PLATFORM_BATV_GAIN_IADC
#error "PLATFORM_BATV_GAIN_IADC undefined (example: iadcCfgAnalogGain0P5x)"
#endif//PLATFORM_BATV_GAIN_IADC

// Set HFRCOEM23 to lowest frequency (1MHz)
#define HFRCOEM23_FREQ          cmuHFRCOEM23Freq_1M0Hz
// Set CLK_ADC to 10kHz (this corresponds to a sample rate of 1ksps)
#define CLK_SRC_ADC_FREQ        1000000 // CLK_SRC_ADC
#define CLK_ADC_FREQ            10000   // CLK_ADC

bool read_battery_voltage (uint16_t * voltage_mV)
{
	if ( ! platform_adc_request (IADC0, 1000))
	{
		return false;
	}

	IADC_Init_t init = IADC_INIT_DEFAULT;
	IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
	IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
	IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;

	// Reset IADC to reset configuration in case it has been modified
	IADC_reset(IADC0);

	// Configure ADC input
	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(PLATFORM_BATV_INPUT_PORT, PLATFORM_BATV_INPUT_PIN, gpioModeDisabled, 0);

	// Enable battery measurement voltage divider (TPS62746)
	GPIO_PinModeSet(PLATFORM_BATV_ENABLE_PORT, PLATFORM_BATV_ENABLE_PIN, gpioModePushPull, PLATFORM_BATV_ENABLE_LEVEL);

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
	initAllConfigs.configs[0].reference = PLATFORM_BATV_REFERENCE_IADC;
	initAllConfigs.configs[0].analogGain = PLATFORM_BATV_GAIN_IADC;

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
	initSingleInput.posInput = PLATFORM_BATV_INPUT_IADC_PORTPIN;
	initSingleInput.negInput = iadcNegInputGnd;

	// Initialize IADC
	IADC_init(IADC0, &init, &initAllConfigs);

	// Initialize Scan
	IADC_initSingle(IADC0, &initSingle, &initSingleInput);

	// Allocate the analog bus for ADC0 inputs
	GPIO->PLATFORM_BATV_INPUT_IADC_BUS |= PLATFORM_BATV_INPUT_IADC_BUSALLOC;

	//debug1("pop %lX", IADC0->SINGLE);

	uint32_t sum = 0;
	uint8_t num = 0;
	for (uint8_t i=0; i<5;i++)
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
	GPIO->PLATFORM_BATV_INPUT_IADC_BUS &= ~(PLATFORM_BATV_INPUT_IADC_BUSALLOC);

	#if 3 == PLATFORM_BATV_DISABLE_LEVEL
		GPIO_PinModeSet(PLATFORM_BATV_ENABLE_PORT, PLATFORM_BATV_ENABLE_PIN, gpioModeDisabled, 0);
	#else
		GPIO_PinModeSet(PLATFORM_BATV_ENABLE_PORT, PLATFORM_BATV_ENABLE_PIN, gpioModePushPull, PLATFORM_BATV_DISABLE_LEVEL);
	#endif

	IADC_reset(IADC0);
	CMU_ClockEnable(cmuClock_IADCCLK, false);

	platform_adc_release(IADC0);

	if (num > 0)
	{
		uint32_t avg = sum / num;
		uint32_t fscale = avg * (PLATFORM_BATV_R_VCC + PLATFORM_BATV_R_GND) / PLATFORM_BATV_R_GND;
		*voltage_mV = PLATFORM_BATV_REFERENCE_VOLTAGE * fscale / (1 << 12); // ADC is 12 bits
		return true;
	}
	return false;
}
