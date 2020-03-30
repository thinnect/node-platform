/*
 * Really basic counter on top of emlib TIMER0.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#include <stdio.h>

#include "em_cmu.h"
#include "em_timer.h"

#include "basic_counter.h"

static volatile uint32_t counter_sec;
static volatile uint16_t counter_msec;

void basic_counter_init() {
	NVIC_DisableIRQ(TIMER0_IRQn);

	counter_sec = 0;
	counter_msec = 0;

	TIMER_Init_TypeDef timerInit = {
		.enable = true,
		.debugRun = false,
		.prescale = timerPrescale1,
		.clkSel = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode = timerModeUp,
		.dmaClrAct = false,
		.quadModeX4 = false,
		.oneShot = false,
		.sync = false,
	};
	TIMER_InitCC_TypeDef timerCCInit = TIMER_INITCC_DEFAULT;

	// Make sure timer uses HFXO as clock source on Series 2
	#if defined(_SILICON_LABS_32B_SERIES_2)
		CMU_ClockSelectSet(cmuClock_EM01GRPACLK, cmuSelect_HFXO);
	#endif
	CMU_ClockEnable(cmuClock_TIMER0, true);

	TIMER_InitCC(TIMER0, 0, &timerCCInit);

	// Set timer top according to the configured clock frequency
	#if defined(_SILICON_LABS_32B_SERIES_2)
		// Previously configured HFXO as the clock source for TIMERs
		TIMER_TopSet(TIMER0, (SystemHFXOClockGet() / 1000) - 1);
	#else
		TIMER_TopSet(TIMER0, (SystemHFClockGet() / 1000) - 1);
	#endif

	TIMER_Init(TIMER0, &timerInit);

	NVIC_SetPriority(TIMER0_IRQn, 0);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);

	TIMER_IntEnable(TIMER0, TIMER_IEN_OF);
}

void basic_counter_deinit() {
	CMU_ClockEnable(cmuClock_TIMER0, false);
}

uint32_t basic_counter_get(uint16_t *msec) {
	uint32_t sec;
	NVIC_DisableIRQ(TIMER0_IRQn);
	sec = counter_sec;
	if(msec != NULL) {
		*msec = counter_msec;
	}
	NVIC_EnableIRQ(TIMER0_IRQn);
	return(sec);
}

void TIMER0_IRQHandler() {
	counter_msec++;
	if(counter_msec == 1000) {
		counter_sec++;
		counter_msec = 0;
	}

	TIMER_IntClear(TIMER0, TIMER_IEN_OF);
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
}
