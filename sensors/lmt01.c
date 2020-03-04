/**
 * Driver for LMT01 from Texas Instruments
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "lmt01.h"
#include <stdint.h>
#include <stdbool.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_prs.h"

static int lmt01_prs_channel = -1;

static void lmt01_acmp_init();
static void lmt01_counter_timer_init();
static uint32_t lmt01_counter_timer_value();
static void lmt01_oneshot_timer_start();
static bool lmt01_oneshot_timer_active();

void lmt01_init()
{
	lmt01_prs_channel = -1;
}

int32_t lmt01_read_temperature()
{
	int32_t counter, temp;

	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_PRS, true);

	if (lmt01_prs_channel < 0)
	{
		lmt01_prs_channel = PRS_GetFreeChannel(prsTypeAsync);
	}

	GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 0);

	lmt01_acmp_init();
	lmt01_counter_timer_init();

	lmt01_oneshot_timer_start();
	while (lmt01_oneshot_timer_active());

	counter = lmt01_counter_timer_value();
	temp = ((counter * 10) / 16) - 500;
	return temp;
}

static void lmt01_acmp_init()
{
	NVIC_DisableIRQ(LMT01_ACMP_IRQ);

	CMU_ClockEnable(LMT01_ACMP_CLOCK, true);

	GPIO_PinModeSet(LMT01_ACMP_INPUT_PORT, LMT01_ACMP_INPUT_PIN, gpioModeInput, 0);

	ACMP_Init_TypeDef acmpInit = ACMP_INIT_DEFAULT;
	ACMP_Init(LMT01_ACMP_DEV, &acmpInit);
	ACMP_ChannelSet(LMT01_ACMP_DEV, acmpInputVREFDIV1V25, LMT01_ACMP_INPUT);
	LMT01_ACMP_INPUT_BUSALLOC = LMT01_ACMP_INPUT_BUSALLOC_STATE;

	PRS_ConnectSignal(lmt01_prs_channel, prsTypeAsync, LMT01_ACMP_PRS_OUT);
}

static void lmt01_counter_timer_init()
{
	NVIC_DisableIRQ(LMT01_COUNTER_TIMER_IRQ);

	CMU_ClockEnable(LMT01_COUNTER_TIMER_CLOCK, true);

	TIMER_InitCC_TypeDef timerCCInit = {
		.edge = timerEdgeRising,
		.prsSel = lmt01_prs_channel,
		.mode = timerCCModeCapture,
		.filter = false,
		.prsInput = true,
		.prsInputType = timerPrsInputAsyncLevel,
		.coist = false,
		.outInvert = false,
	};
	TIMER_InitCC(LMT01_COUNTER_TIMER_DEV, 1, &timerCCInit);

	TIMER_Init_TypeDef timerInit = {
		.enable = true,
		.debugRun = false,
		.prescale = timerPrescale1,
		.clkSel = timerClkSelCC1,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode = timerModeUp,
		.dmaClrAct = false,
		.quadModeX4 = false,
		.oneShot = false,
		.sync = false,
	};
	TIMER_Init(LMT01_COUNTER_TIMER_DEV, &timerInit);

	PRS_ConnectConsumer(lmt01_prs_channel, prsTypeAsync, LMT01_COUNTER_TIMER_PRS_CC1);
}

static uint32_t lmt01_counter_timer_value()
{
	return TIMER_CounterGet(LMT01_COUNTER_TIMER_DEV);
}

static void lmt01_oneshot_timer_start()
{
	uint32_t topvalue;
	NVIC_DisableIRQ(LMT01_ONESHOT_TIMER_IRQ);

	CMU_ClockEnable(LMT01_ONESHOT_TIMER_CLOCK, true);

	TIMER_InitCC_TypeDef timerCCInit = {
		.eventCtrl = timerEventEveryEdge,
		.edge = timerEdgeBoth,
		.cufoa = timerOutputActionNone,
		.cofoa = timerOutputActionClear,
		.cmoa = timerOutputActionSet,
		.mode = timerCCModeCompare,
		.filter = false,
		.prsInput = false,
		.coist = false,
		.outInvert = false,
	};
	TIMER_InitCC(LMT01_ONESHOT_TIMER_DEV, 0, &timerCCInit);

	GPIO->TIMERROUTE[LMT01_ONESHOT_TIMER_INDEX].CC0ROUTE = (LMT01_ONESHOT_TIMER_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
		| (LMT01_ONESHOT_TIMER_PIN << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
	GPIO->TIMERROUTE[LMT01_ONESHOT_TIMER_INDEX].ROUTEEN = GPIO_TIMER_ROUTEEN_CC0PEN;

	topvalue = ((LMT01_ONESHOT_TIMER_PERIPHERAL_CLOCK / 256LU) / 15LU) - 1LU;
	TIMER_TopSet(LMT01_ONESHOT_TIMER_DEV, topvalue);
	TIMER_CompareSet(LMT01_ONESHOT_TIMER_DEV, 0, 1);

	TIMER_Init_TypeDef timerInit = {
		.enable = true,
		.debugRun = false,
		.prescale = timerPrescale256,
		.clkSel = timerClkSelHFPerClk,
		.fallAction = timerInputActionNone,
		.riseAction = timerInputActionNone,
		.mode = timerModeUp,
		.dmaClrAct = false,
		.quadModeX4 = false,
		.oneShot = true,
		.sync = false,
	};
	TIMER_IntClear(LMT01_ONESHOT_TIMER_DEV, TIMER_IEN_OF);
	TIMER_Init(LMT01_ONESHOT_TIMER_DEV, &timerInit);
}

static bool lmt01_oneshot_timer_active()
{
	return (!(TIMER_IntGet(LMT01_ONESHOT_TIMER_DEV) & TIMER_IF_OF));
}

