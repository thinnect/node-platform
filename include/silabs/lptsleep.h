/**
 * Low Power Sleep Mode setup header.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 */
#ifndef LPTSLEEP_H_
#define LPTSLEEP_H_

#include "em_cmu.h"
#include "sleep.h"

// Important configuration parameters:
// configLPTSLEEP_RTCC_CHANNEL
//    The RTCC channel reserved for sleep mode - 2
// configLPTSLEEP_RTCC_IF
//    The IF mask according to the channel used - RTCC_IF_CC2
// configLPTSLEEP_RTCC_IEN
//    The IEN mask according to the channel used - RTCC_IEN_CC2
// configLPTSLEEP_USE_EXT_IRQ
//    Indicate that the RTCC IRQ is defined elsewhere

/**
 * Enable sleep mode according to compile-time configuration.
 * @param block Block this energy mode and anything deeper.
 *   sleepEM1, sleepEM2 or sleepEM3, maybe sleepEM4
 */
void vLowPowerSleepModeSetup(SLEEP_EnergyMode_t block);

/**
 * The timer used for sleep needs to be configured, but not necessarily by the
 * sleep component - sleep mode only requires that it run
 * continuously at the max rate (32.768kHz) and that one compare channel is
 * dedicated for it (defined with configLPTSLEEP_RTCC_CHANNEL).
 * @param osc cmuSelect_LFRCO or cmuSelect_LFXO.
 */
void vLowPowerSleepTimerSetup(CMU_Select_TypeDef osc);

/**
 * Get the current frequency of the sleep timer. Ideally this would be
 * 32768, but not all clocks are accurate and the calibration may have been
 * changed.
 *
 * @return current frequency.
*/
uint32_t ulLowPowerSleepTimerGetFreq();

/**
 * Specify the actual frequency - ideally would be 32768, but can be set
 * with this function if it is not.
 * @param freq - Actual frequency of the timer used for sleep.
 */
void vLowPowerSleepTimerSetFreq(uint32_t freq);

/**
 * Get the time spent in sleep - milliseconds.
 * @return Time spent sleeping.
 */
uint32_t ulLowPowerSleepTime();

#endif//LPTSLEEP_H_
