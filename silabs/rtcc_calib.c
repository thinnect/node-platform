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

#ifndef RTCC_CALIB_STABIL_S
#define RTCC_CALIB_STABIL_S 10
#endif//RTCC_CALIB_STABIL_S

#ifndef RTCC_CALIB_TIME_S
#define RTCC_CALIB_TIME_S 60
#endif//RTCC_CALIB_TIME_S

#if defined(_SILICON_LABS_32B_SERIES_2)
#define RTCC_CALIB_REFERENCE TIMER0
#define RTCC_CALIB_REFERENCE_CLOCK cmuClock_TIMER0
#else
#define RTCC_CALIB_REFERENCE WTIMER0
#define RTCC_CALIB_REFERENCE_CLOCK cmuClock_WTIMER0
#endif

static uint32_t m_period_ms;
static freq_set_f * m_setfunc;
static osThreadId_t m_calib_thread;

static void ref_timer_enable ()
{
    //NVIC_DisableIRQ(WTIMER0_IRQn);

    TIMER_Init_TypeDef timerInit =
    {
        .enable = false,
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
    CMU_ClockEnable(RTCC_CALIB_REFERENCE_CLOCK, true);

    // TIMER_Reset(RTCC_CALIB_REFERENCE);
    TIMER_Init(RTCC_CALIB_REFERENCE, &timerInit);
    TIMER_TopSet(RTCC_CALIB_REFERENCE, 0xFFFFFFFF);
    TIMER_Enable(RTCC_CALIB_REFERENCE, true);
}

static uint32_t ref_timer_get ()
{
    return TIMER_CounterGet(RTCC_CALIB_REFERENCE);
}

static void ref_timer_disable ()
{
    TIMER_Enable(RTCC_CALIB_REFERENCE, false);
}

uint32_t rtcc_calibrate_now ()
{
    ref_timer_enable();
    debug1("stblzng");

    watchdog_feed();
    while(ref_timer_get() < RTCC_CALIB_STABIL_S * 37500UL);

    debug1("clbrtng");
    uint32_t tmr = ref_timer_get();
    uint32_t rtc = RTCC_CounterGet();

    for(uint8_t i=0;i<=RTCC_CALIB_TIME_S;i+=1)
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

void rtcc_disable_calibration ()
{
    if (NULL != m_calib_thread)
    {
        osThreadFlagsSet(m_calib_thread, 1);
        // osThreadJoin(m_calib_thread); // not supported on all CMSIS versions
        while (osThreadTerminated != osThreadGetState(m_calib_thread))
        {
            osDelay(10);
        }
        m_calib_thread = NULL;
    }
    else
    {
        ref_timer_disable();
    }

    TIMER_Reset(RTCC_CALIB_REFERENCE);

    debug1("dsbld");
}

static void calibration_loop (void *arg)
{
    for(;;)
    {
        uint32_t flags = osThreadFlagsWait(1, osFlagsWaitAny, m_period_ms);
        if (flags == osFlagsErrorTimeout)
        {
            SLEEP_SleepBlockBegin(sleepEM2); // Must block sleep modes that stop the reference timer

            ref_timer_enable();
            debug1("stblzng");

            osDelay(RTCC_CALIB_STABIL_S*1000); // Wait RTCC_CALIB_STABIL_S seconds for things to stabilize

            debug1("clbrtng");

            vPortEnterCritical();
            uint32_t tmr = ref_timer_get();
            uint32_t rtc = RTCC_CounterGet();
            vPortExitCritical();

            osDelay(RTCC_CALIB_TIME_S*1000); // Run both timers for RTCC_CALIB_TIME_S seconds

            vPortEnterCritical();
            uint32_t etmr = ref_timer_get();
            uint32_t ertc = RTCC_CounterGet();
            vPortExitCritical();

            ref_timer_disable();
            SLEEP_SleepBlockEnd(sleepEM2);

            uint32_t freq = (uint32_t)(((uint64_t)37500)*(ertc - rtc)/(etmr - tmr));
            debug1("rtc: %u/%u tmr: %u/%u freq %"PRIu32, rtc, ertc, tmr, etmr, freq);

            if (NULL != m_setfunc)
            {
                m_setfunc(freq);
            }
        }
        else
        {
            debug1("exit");
            osThreadExit();
        }
    }
}

// Do this before starting the OS and do this only once!
void rtcc_configure_periodic_calibration (uint32_t period_ms, freq_set_f * setfunc)
{
    m_period_ms = period_ms;
    m_setfunc = setfunc;

    const osThreadAttr_t calib_thread_attr = { .name = "calib", .stack_size = 1024 };
    m_calib_thread = osThreadNew(calibration_loop, NULL, &calib_thread_attr);
}
