/*
 * Customized low-power tickless idle implementation. To be used together with
 * the Cortex M3 port.c in the FreeRTOS kernel.
 *
 * The implementation uses SysTick during active periods and uses the RTCC to
 * account for the sleep period.
 *
 * Based on parts of EFM32 low_power_tick_management_RTCC.c and CM3 port.c from
 * the FreeRTOS kernel.
 *
 * Copyright Thinnect Inc. 2019
 * Modifications licensed under the same terms as the original FreeRTOS files.
 */

/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <limits.h>

#include "FreeRTOS.h"
#include "task.h"

#include <stdbool.h>

#include "em_cmu.h"
#include "em_rtcc.h"
#include "sleep.h"
#include "em_rmu.h"
#include "em_core.h"

#include "em_gpio.h"

#include "loglevels.h"
#define __MODUUL__ "lpts"
#define __LOG_LEVEL__ (LOG_LEVEL_lptsleep & BASE_LOG_LEVEL)
#include "log.h"

//------------------------------------------------------------------------------
// Duplicate some parameters from port.c that are not accessible globally
// These may not be universal!!!
//------------------------------------------------------------------------------

/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG           ( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG           ( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG  ( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG                ( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_INT_BIT            ( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT         ( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT     ( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT            ( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT     ( 1UL << 25UL )

#define portNVIC_PENDSV_PRI                 ( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 16UL )
#define portNVIC_SYSTICK_PRI                ( ( ( uint32_t ) configKERNEL_INTERRUPT_PRIORITY ) << 24UL )

#ifndef configSYSTICK_CLOCK_HZ
    #define configSYSTICK_CLOCK_HZ configCPU_CLOCK_HZ
    /* Ensure the SysTick is clocked at the same frequency as the core. */
    #define portNVIC_SYSTICK_CLK_BIT    ( 1UL << 2UL )
#else
    /* The way the SysTick is clocked is not modified in case it is not the same
    as the core. */
    #define portNVIC_SYSTICK_CLK_BIT    ( 0 )
#endif

//------------------------------------------------------------------------------

// Count the time spent in sleep, milliseconds
static uint32_t ulTotalSleepTime = 0;

static TickType_t xMaximumPossibleSuppressedTicks = 0;

static uint32_t ulTimerCountsForOneTick = 0;
static uint32_t ulStoppedTimerCompensation = 0; // TODO determine an actual value

// RTCC configuration structures
static RTCC_Init_TypeDef xRTCInitStruct =
{
  false,                /* Start counting when the initialization is complete. */
  false,                /* Disable updating RTC during debug halt. */
  false,                /* Prescaler counts to maximum before wrapping around. */
  false,                /* Counter counts to maximum before wrapping around. */
  rtccCntPresc_1,       /* Set RTCC prescaler to 1. */
  rtccCntTickPresc,     /* Count according to the prescaler configuration. */
#if defined(_RTCC_CTRL_BUMODETSEN_MASK)
  false,                /* Disable storing RTCC counter value in RTCC_CCV2 upon backup mode entry. */
#endif
#if defined(_RTCC_CTRL_OSCFDETEN_MASK)
  false,                /* LFXO fail detection disabled */
#endif
#if defined (_RTCC_CTRL_CNTMODE_MASK)
  rtccCntModeNormal,    /* Use RTCC in normal mode and not in calender mode */
#endif
#if defined (_RTCC_CTRL_LYEARCORRDIS_MASK)
  false                 /* No leap year correction. */
#endif
};

static const RTCC_CCChConf_TypeDef xRTCCChannel1InitStruct =
{
    rtccCapComChModeCompare,    /* Use Compare mode. */
    rtccCompMatchOutActionPulse,/* Don't care. */
    rtccPRSCh0,                 /* PRS not used. */
    rtccInEdgeNone,             /* Capture Input not used. */
    rtccCompBaseCnt,            /* Compare with Base CNT register. */
#if defined (_RTCC_CC_CTRL_COMPMASK_MASK)
    0,
#endif
#if defined (_RTCC_CC_CTRL_DAYCC_MASK)
    rtccDayCompareModeMonth     /* Don't care. */
#endif
};

// A compare channel and interrupt flag mask need to be dedicated for sleep.
#ifndef configLPTSLEEP_RTCC_CHANNEL
#define configLPTSLEEP_RTCC_CHANNEL (2)
#endif//configLPTSLEEP_RTCC_CHANNEL
#ifndef configLPTSLEEP_RTCC_IF
#define configLPTSLEEP_RTCC_IF RTCC_IF_CC2
#endif//configLPTSLEEP_RTCC_IF
#ifndef configLPTSLEEP_RTCC_IEN
#define configLPTSLEEP_RTCC_IEN RTCC_IEN_CC2
#endif//configLPTSLEEP_RTCC_IEN

// The sleep timer frequency may be adjusted ~ calibrated
static uint32_t ulSleepTIMER_FREQUENCY_HZ = 32768UL;

//------------------------------------------------------------------------------

void vLowPowerSleepModeSetup (SLEEP_EnergyMode_t block)
{
    xMaximumPossibleSuppressedTicks = ULONG_MAX / (ulSleepTIMER_FREQUENCY_HZ / configTICK_RATE_HZ);
    ulTimerCountsForOneTick = ( configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ );

    debug1("ulTimerCountsForOneTick %d", (unsigned int)ulTimerCountsForOneTick);

    // Set the sleep mode, assumes SLEEP_Init has been performed already!
    SLEEP_SleepBlockBegin(block);
}

/**
 * Timer setup here may not make sense - sleep mode only requires that it run
 * continuously at the max rate (32.768kHz) and that on compare channel is
 * dedicated for it (defined with configLPTSLEEP_RTCC_CHANNEL).
 * @param osc cmuSelect_LFRCO or cmuSelect_LFXO
 */
void vLowPowerSleepTimerSetup (CMU_Select_TypeDef osc)
{
    /* Ensure LE modules are accessible. */
    #if defined(cmuClock_CORELE)
        CMU_ClockEnable(cmuClock_CORELE, true);
    #endif

    #if defined(CMU_LFECLKEN0_RTCC) // Series 1?
        // Enable LFECLK in CMU (will also enable oscillator if not enabled).
        CMU_ClockSelectSet(cmuClock_LFE, osc);
    #else
        #if defined(_SILICON_LABS_32B_SERIES_2)
            CMU_ClockSelectSet(cmuClock_RTCC, osc);
        #else // Series 0?
            // Enable LFACLK in CMU (will also enable oscillator if not enabled).
            CMU_ClockSelectSet(cmuClock_LFA, osc);
        #endif
    #endif

    /* Enable clock to the RTC module. */
    CMU_ClockEnable(cmuClock_RTCC, true);

    RTCC_Init(&xRTCInitStruct);
    RTCC_ChannelInit(configLPTSLEEP_RTCC_CHANNEL, &xRTCCChannel1InitStruct);
    #if defined (RTCC_EM4WUEN_EM4WU)
        RTCC_EM4WakeupEnable( true );
    #endif//RTCC_EM4WUEN_EM4WU

    /* Disable RTCC interrupt. */
    RTCC_IntDisable(_RTCC_IF_MASK);
    RTCC_IntClear(_RTCC_IF_MASK);
    RTCC->CNT = _RTCC_CNT_RESETVALUE;

    /* The tick interrupt must be set to the lowest priority possible. */
    NVIC_SetPriority(RTCC_IRQn, 7); // FIXME: 7 is a random number here
    NVIC_ClearPendingIRQ(RTCC_IRQn);
    NVIC_EnableIRQ(RTCC_IRQn);

    RTCC_Enable(true);

    #if defined(_SILICON_LABS_32B_SERIES_2)
        RTCC_SyncWait();
    #endif

    debug1("rtcc initd");
}

/**
 * Get the current frequency.
 */
uint32_t ulLowPowerSleepTimerGetFreq ()
{
    uint32_t freq;
    vPortEnterCritical();
    freq = ulSleepTIMER_FREQUENCY_HZ;
    vPortExitCritical();
    return freq;
}

/**
 * Specify the actual frequency - ideally would be 32768, but set it with this
 * function if it is not!
 * @param freq - Actual frequency of the timer used for sleep.
 */
void vLowPowerSleepTimerSetFreq (uint32_t freq)
{
    vPortEnterCritical();
    ulSleepTIMER_FREQUENCY_HZ = freq;
    vPortExitCritical();
}

/**
 * Get total time spent in sleep.
 */
uint32_t ulLowPowerSleepTime ()
{
    return ulTotalSleepTime;
}

/**
 * Provide an override to the default weakly defined vPortSuppressTicksAndSleep
 * function.
 */
void vPortSuppressTicksAndSleep (TickType_t xExpectedIdleTime)
{
    uint32_t ulReloadValue;
    uint32_t ulCompleteTickPeriods;
    TickType_t xModifiableIdleTime;

    CORE_DECLARE_IRQ_STATE;

    if (xExpectedIdleTime < 2)
    {
        return; // Must always sleep xExpectedIdleTime - 1 and cannot handle 0.
    }

    // Make sure the sleep value does not overflow the counter.
    if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    // Stop the SysTick momentarily.  The time the SysTick is stopped for
    //is accounted for as best it can be, but using the tickless mode will
    //inevitably result in some tiny drift of the time maintained by the
    //kernel with respect to calendar time.
    portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

    ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG;
    if (ulReloadValue > ulStoppedTimerCompensation)
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    // Enter a critical section but don't use the taskENTER_CRITICAL()
    // method as that will mask interrupts that should exit sleep mode.
    CORE_ENTER_CRITICAL();
    //__asm volatile( "cpsid i" ::: "memory" );
    __asm volatile( "dsb" );
    __asm volatile( "isb" );

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if (eTaskConfirmSleepModeStatus() == eAbortSleep)
    {
        /* Restart from whatever is left in the count register to complete
        this tick period. */
        portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

        /* Reset the reload register to the value required for normal tick
        periods. */
        portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

        // Re-enable interrupts.
        //__asm volatile( "cpsie i" ::: "memory" );
        CORE_EXIT_CRITICAL();
    }
    else
    {
        // Calculate sleep value in RTCC units, cannot sleep the full period as
        // we cannot generate the actual tick interrupt, so -1.
        uint32_t ulCountAfterSleep;
        uint32_t ulCountBeforeSleep = RTCC_CounterGet();
        uint32_t ulSleepValue = (ulSleepTIMER_FREQUENCY_HZ * (xExpectedIdleTime - 1)) / configTICK_RATE_HZ;
        uint32_t ulSleepCount;

        RTCC_ChannelCCVSet(configLPTSLEEP_RTCC_CHANNEL, ulCountBeforeSleep+ulSleepValue);
        RTCC_IntEnable(configLPTSLEEP_RTCC_IEN);

        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING( xModifiableIdleTime );
        if (xModifiableIdleTime > 0)
        {
            __asm volatile( "dsb" ::: "memory" );
            SLEEP_Sleep();
            __asm volatile( "isb" );
        }
        configPOST_SLEEP_PROCESSING( xExpectedIdleTime );

        // Disable the interrupt
        RTCC_IntDisable(configLPTSLEEP_RTCC_IEN);
        // Clear the interrupt flag of the timer and don't let it run at all
        RTCC_IntClear(configLPTSLEEP_RTCC_IF);
        // If the RTCC IRQ is defined in this module, clear the pending IRQ and don't let it run at all
        #ifndef configLPTSLEEP_USE_EXT_IRQ
        NVIC_ClearPendingIRQ(RTCC_IRQn);
        #endif//configLPTSLEEP_USE_EXT_IRQ

        __asm volatile( "dsb" ::: "memory" );
        __asm volatile( "isb" );

        ulCountAfterSleep = RTCC_CounterGet();
        ulSleepCount = ulCountAfterSleep - ulCountBeforeSleep;
        ulCompleteTickPeriods = (ulSleepCount * configTICK_RATE_HZ) / ulSleepTIMER_FREQUENCY_HZ;

        ulTotalSleepTime += ulCompleteTickPeriods;

        uint32_t ulExtraSleepTicks = ulSleepCount - ulCompleteTickPeriods * ulSleepTIMER_FREQUENCY_HZ / configTICK_RATE_HZ;
        uint32_t ulExtraSysTicks = (ulTimerCountsForOneTick*ulExtraSleepTicks * configTICK_RATE_HZ) / ulSleepTIMER_FREQUENCY_HZ;

        if (ulExtraSysTicks < ulReloadValue)
        {
            ulReloadValue -= ulExtraSysTicks;
        }
        else // Slept a bit too long, do a tick ASAP and hope it's ok
        {
            ulReloadValue = 1;
        }

        portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

        // Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
        // again, then set portNVIC_SYSTICK_LOAD_REG back to its standard value.
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
        vTaskStepTick( ulCompleteTickPeriods );
        portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

        /* Exit with interrupts enabled. */
        //__asm volatile( "cpsie i" ::: "memory" );
        CORE_EXIT_CRITICAL();
    }
}

/**
 * The RTCC interrupt serves only as a wakeup and the flag itself is cleared in
 * the sleep function. The handler may be defined and used elsewhere and the
 * interrupt definition must then be disabled in configuration with
 * configLPTSLEEP_USE_EXT_IRQ. Disabling the handler here and not defining
 * another one elsewhere will result in it being defined as the Default Handler.
 *
 * This handler normally never executes.
 */
#ifndef configLPTSLEEP_USE_EXT_IRQ
void RTCC_IRQHandler (void)
{
	RTCC_IntClear(_RTCC_IF_MASK); // Clear all interrupts
}
#endif//configLPTSLEEP_USE_EXT_IRQ
