/**
 * An RTCC calibration solution developed compensate the lack of an accurate
 * low-frequency oscillator on the tsb0 device. The frequency of the LFRCO
 * is determined through a comparison of the RTCC running on the LFRCO and a
 * Timer running on HFXO. It does not actually configure the RTCC, so it is
 * possible to run the "calibration" also for LFXO and other boards, to perform
 * some comparisons and such.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "rtcc_calib.h"

#include <inttypes.h>

#include "em_timer.h"
#include "em_rtcc.h"
#include "em_cmu.h"
#include "sleep.h"

#include "cmsis_os2.h"

#include "watchdog.h"

extern void vPortEnterCritical();
extern void vPortExitCritical();

#include "loglevels.h"
#define __MODUUL__ "calib"
#define __LOG_LEVEL__ (LOG_LEVEL_rtcc_calib & BASE_LOG_LEVEL)
#include "log.h"

#if defined(_SILICON_LABS_32B_SERIES_2)
#define RTCC_TEST_REFERENCE TIMER0
#define RTCC_TEST_REFERENCE_CLOCK cmuClock_TIMER0
#else
#define RTCC_TEST_REFERENCE WTIMER0
#define RTCC_TEST_REFERENCE_CLOCK cmuClock_WTIMER0
#endif

static uint32_t m_period_ms;
static freq_set_f * m_setfunc;

static void ref_timer_enable ()
{
    //NVIC_DisableIRQ(WTIMER0_IRQn);

    TIMER_Init_TypeDef timerInit =
    {
        .enable = true,
        .debugRun = false,
        .prescale = timerPrescale1024,
        .clkSel = timerClkSelHFPerClk,
        .fallAction = timerInputActionNone,
        .riseAction = timerInputActionNone,
        .mode = timerModeUp,
        .dmaClrAct = false,
        .quadModeX4 = false,
        .oneShot = false,
        .sync = false,
    };

    #if defined(_SILICON_LABS_32B_SERIES_2)
        CMU_ClockSelectSet(cmuClock_EM01GRPACLK, cmuSelect_HFXO);
    #endif
    CMU_ClockEnable(RTCC_TEST_REFERENCE_CLOCK, true);

    TIMER_Init(RTCC_TEST_REFERENCE, &timerInit);
    TIMER_TopSet(RTCC_TEST_REFERENCE, 0xFFFFFFFF);
}

static uint32_t ref_timer_get ()
{
    return TIMER_CounterGet(RTCC_TEST_REFERENCE);
}

static void ref_timer_disable ()
{
    TIMER_Enable(RTCC_TEST_REFERENCE, false);
}

uint32_t rtcc_calibrate_now ()
{
    ref_timer_enable();
    debug1("stblzng");

    watchdog_feed();
    while(ref_timer_get() < 10 * 37500UL);

    debug1("clbrtng");
    uint32_t tmr = ref_timer_get();
    uint32_t rtc = RTCC_CounterGet();

    for(uint8_t i=0;i<=60;i+=10)
    {
        watchdog_feed();
        while(ref_timer_get() - tmr < i * 37500UL);
        debug1("%d", i);
    }
    watchdog_feed();

    uint32_t etmr = ref_timer_get();
    uint32_t ertc = RTCC_CounterGet();

    ref_timer_disable();

    uint32_t freq = (uint32_t)(((uint64_t)37500)*(ertc - rtc)/(etmr - tmr));
    debug1("rtc: %u/%u tmr: %u/%u freq %"PRIu32, rtc, ertc, tmr, etmr, freq);

    return freq;
}

static void calibration_loop (void *arg)
{
    for(;;)
    {
        osDelay(m_period_ms);

        ref_timer_enable();
        debug1("stblzng");

        SLEEP_SleepBlockBegin(sleepEM1); // Must block sleep modes that stop the reference timer

        osDelay(10000); // Wait 10 seconds for things to stabilize

        debug1("clbrtng");

        vPortEnterCritical();
        uint32_t tmr = ref_timer_get();
        uint32_t rtc = RTCC_CounterGet();
        vPortExitCritical();

        osDelay(60000); // Run both timers for 60 seconds

        vPortEnterCritical();
        uint32_t etmr = ref_timer_get();
        uint32_t ertc = RTCC_CounterGet();
        vPortExitCritical();

        ref_timer_disable();
        SLEEP_SleepBlockEnd(sleepEM1);

        uint32_t freq = (uint32_t)(((uint64_t)37500)*(ertc - rtc)/(etmr - tmr));
        debug1("rtc: %u/%u tmr: %u/%u freq %"PRIu32, rtc, ertc, tmr, etmr, freq);

        m_setfunc(freq);
    }
}

// Do this before starting the OS and do this only once!
void rtcc_configure_periodic_calibration (uint32_t period_ms, freq_set_f * setfunc)
{
    m_period_ms = period_ms;
    m_setfunc = setfunc;

    const osThreadAttr_t calib_thread_attr = { .name = "calib" };
    osThreadNew(calibration_loop, NULL, &calib_thread_attr);
}
