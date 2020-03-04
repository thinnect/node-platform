/**
 * Driver for LMT01 from Texas Instruments
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#ifndef _LMT01_H_
#define _LMT01_H_

#include <stdint.h>

#define LMT01_ACMP_DEV                       ACMP0
#define LMT01_ACMP_CLOCK                     cmuClock_ACMP0
#define LMT01_ACMP_IRQ                       ACMP0_IRQn
#define LMT01_ACMP_INPUT                     acmpInputPB0
#define LMT01_ACMP_INPUT_PORT                gpioPortB
#define LMT01_ACMP_INPUT_PIN                 0
#define LMT01_ACMP_INPUT_BUSALLOC            (GPIO->BBUSALLOC)
#define LMT01_ACMP_INPUT_BUSALLOC_STATE      GPIO_BBUSALLOC_BEVEN0_ACMP0
#define LMT01_ACMP_PRS_OUT                   prsSignalACMP0_OUT

#define LMT01_COUNTER_TIMER_DEV              TIMER2
#define LMT01_COUNTER_TIMER_CLOCK            cmuClock_TIMER2
#define LMT01_COUNTER_TIMER_IRQ              TIMER2_IRQn
#define LMT01_COUNTER_TIMER_PRS_CC1          prsConsumerTIMER2_CC1

#define LMT01_ONESHOT_TIMER_DEV              TIMER1
#define LMT01_ONESHOT_TIMER_CLOCK            cmuClock_TIMER1
#define LMT01_ONESHOT_TIMER_IRQ              TIMER1_IRQn
#define LMT01_ONESHOT_TIMER_INDEX            1
#define LMT01_ONESHOT_TIMER_PERIPHERAL_CLOCK 38400000LU
#define LMT01_ONESHOT_TIMER_PORT             gpioPortA
#define LMT01_ONESHOT_TIMER_PIN              6

/**
 * Initializes LMT01 driver
 */
void lmt01_init();

/**
 * Read temperature from LMT01
 *
 * @return Returns temperature in C * 10
 */
int32_t lmt01_read_temperature();

#endif

