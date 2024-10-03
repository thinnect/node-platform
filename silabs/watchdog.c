/*
 * MCU watchdog implementation.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Konstantin Bilozor
 */
#include <inttypes.h>
#include "watchdog.h"

#include "em_wdog.h"
#include "em_cmu.h"

#include "loglevels.h"
#define __MODUUL__ "wdog"
#define __LOG_LEVEL__ (LOG_LEVEL_wdog & BASE_LOG_LEVEL)
#include "log.h"


static uint32_t wdogTimeout;
static uint32_t warnPeriod;

static WDOG_Init_TypeDef wdogInit = WDOG_INIT_DEFAULT;
static watchdog_warning_f* warnCb;


void WDOG0_IRQHandler(void) {
    uint32_t flags;
    flags = WDOGn_IntGet(DEFAULT_WDOG);
    if (flags & WDOG_IEN_WARN) {
        WDOGn_IntClear(DEFAULT_WDOG, WDOG_IEN_WARN);
        warnCb(wdogTimeout-warnPeriod);
    }
}


void watchdog_feed() {
    WDOGn_Feed(DEFAULT_WDOG);
}


// Clock periods are from WDOG_PeriodSel_TypeDef in em_wdog.h file
WDOG_PeriodSel_TypeDef wdog_periodSelect(uint32_t timeout_ms) {
    if (timeout_ms < 9) {
        wdogTimeout = 9;
        return wdogPeriod_9;
    } else if (timeout_ms < 17) {
        wdogTimeout = 17;
        return wdogPeriod_17;
    } else if (timeout_ms < 33) {
        wdogTimeout = 33;
        return wdogPeriod_33;
    } else if (timeout_ms < 65) {
        wdogTimeout = 65;
        return wdogPeriod_65;
    } else if (timeout_ms < 129) {
        wdogTimeout = 129;
        return wdogPeriod_129;
    } else if (timeout_ms < 257) {
        wdogTimeout = 257;
        return wdogPeriod_257;
    } else if (timeout_ms < 513) {
        wdogTimeout = 513;
        return wdogPeriod_513;
    } else if (timeout_ms < 1025) {
        wdogTimeout = 1025;
        return wdogPeriod_1k;
    } else if (timeout_ms < 2049) {
        wdogTimeout = 2049;
        return wdogPeriod_2k;
    } else if (timeout_ms < 4097) {
        wdogTimeout = 4097;
        return wdogPeriod_4k;
    } else if (timeout_ms < 8193) {
        wdogTimeout = 8193;
        return wdogPeriod_8k;
    } else if (timeout_ms < 16385) {
        wdogTimeout = 16385;
        return wdogPeriod_16k;
    } else if (timeout_ms < 32769) {
        wdogTimeout = 32769;
        return wdogPeriod_32k;
    } else if (timeout_ms < 65537) {
        wdogTimeout = 65537;
        return wdogPeriod_64k;
    } else if (timeout_ms < 131073) {
        wdogTimeout = 131073;
        return wdogPeriod_128k;
    } else if (timeout_ms < 262145) {
        wdogTimeout = 262145;
        return wdogPeriod_256k;
    } else {
        err1("watchdog period is too long!");
        return -1;
    }
}


bool watchdog_enable(uint32_t timeout_ms) {

    debug1("Enable: %u");
    WDOG_PeriodSel_TypeDef per;
#ifdef EFR32_SERIES1
    CMU_ClockEnable(cmuClock_CORELE, true);
    wdogInit.clkSel = wdogClkSelULFRCO;
#elif EFR32_SERIES2
    CMU_ClockSelectSet(cmuClock_WDOG0, cmuSelect_ULFRCO);
#else
    #error("MCU series are not set. Select which series MCU is used!");
#endif

    wdogInit.debugRun = true;
    wdogInit.em2Run = true;

    per = wdog_periodSelect(timeout_ms);
    debug1("wdog per:%u", per);
    if (per >= wdogPeriod_9) {
        wdogInit.perSel = per;
    } else {
        return false;
    }

    WDOGn_Init(DEFAULT_WDOG, &wdogInit);
    return true;
}


uint32_t watchdog_period() {
    if(WDOGn_IsEnabled(DEFAULT_WDOG)) {
        return wdogTimeout;
    } else {
        return UINT32_MAX;
    }
}


void watchdog_disable() {
    WDOGn_Enable(DEFAULT_WDOG, false);
}


bool watchdog_enable_warning(uint32_t timeout_ms, watchdog_warning_f* callback) {
#ifdef WDOG_WARN_NOT_PRESENT
    return false;
#else
    uint8_t ratioPct;

    if (wdogTimeout < timeout_ms) {
        warn1("Warning must be smaller than watchdog period");
        return false;
    }

    ratioPct = (timeout_ms*100) / wdogTimeout;

    if (ratioPct < 25) {
        wdogInit.warnSel = wdogWarnTime25pct;
        warnPeriod = wdogTimeout >> 2;
    } else if (ratioPct < 50) {
        wdogInit.warnSel = wdogWarnTime50pct;
        warnPeriod = wdogTimeout >> 1;
    } else {
        wdogInit.warnSel = wdogWarnTime75pct;
        warnPeriod = (wdogTimeout >> 2) * 3;
    }

    debug1("warnPeriod: %"PRIu32, warnPeriod);

    WDOG_Init(&wdogInit);

    warnCb = callback;

    WDOGn_IntEnable(DEFAULT_WDOG, WDOG_IEN_WARN);
    NVIC_ClearPendingIRQ(WDOG0_IRQn);
    NVIC_EnableIRQ(WDOG0_IRQn);
    return true;
#endif
}


uint32_t watchdog_warning_period() {
    if (WDOGn_IntGetEnabled(DEFAULT_WDOG) == WDOG_IEN_WARN) {
        return UINT32_MAX;
    } else {
        return warnPeriod;
    }
}


void watchdog_disable_warning() {
    WDOGn_IntDisable(DEFAULT_WDOG, WDOG_IEN_WARN);
    NVIC_DisableIRQ(WDOG0_IRQn);
}
