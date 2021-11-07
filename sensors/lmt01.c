/**
 * Driver for LMT01 from Texas Instruments
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "lmt01.h"
#include "lmt01_config.h"
#include <stdint.h>
#include <stdbool.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_timer.h"
#include "em_acmp.h"
#include "em_prs.h"
#include "platform_io.h"

#ifdef _SILICON_LABS_32B_SERIES_2

#ifndef ACMP_TRESHOLD
#define ACMP_TRESHOLD acmpInputVREFDIV1V25
#endif//ACMP_TRESHOLD
#ifndef LMT01_ONESHOT_TIMER_PORT
#define LMT01_ONESHOT_TIMER_PORT A
#endif//LMT01_ONESHOT_TIMER_PORT
#ifndef LMT01_ONESHOT_TIMER_PIN
#define LMT01_ONESHOT_TIMER_PIN 6
#endif//LMT01_ONESHOT_TIMER_PIN

#define ___LMT01_STRINGIFY_PRE2(__p1, __p2)             __p1 ## __p2
#define ___LMT01_STRINGIFY_PRE3(__p1, __p2, __p3)       __p1 ## __p2 ## __p3
#define ___LMT01_STRINGIFY_PRE4(__p1, __p2, __p3, __p4) __p1 ## __p2 ## __p3 ## __p4
#define ___LMT01_STRINGIFY2(__p1, __p2)                 ___LMT01_STRINGIFY_PRE2(__p1, __p2)
#define ___LMT01_STRINGIFY3(__p1, __p2, __p3)           ___LMT01_STRINGIFY_PRE3(__p1, __p2, __p3)
#define ___LMT01_STRINGIFY4(__p1, __p2, __p3, __p4)     ___LMT01_STRINGIFY_PRE4(__p1, __p2, __p3, __p4)

#define LMT01_ACMP_DEV                    (___LMT01_STRINGIFY2(ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_CLOCK                  (___LMT01_STRINGIFY2(cmuClock_ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_IRQ                    (___LMT01_STRINGIFY3(ACMP, LMT01_ACMP_DEV_INDEX, _IRQn))
#define LMT01_ACMP_INPUT                  (___LMT01_STRINGIFY3(acmpInputP, LMT01_ACMP_INPUT_PORT, LMT01_ACMP_INPUT_PIN))
#define LMT01_ACMP_INPUT_GPIO_PORT        (___LMT01_STRINGIFY2(gpioPort, LMT01_ACMP_INPUT_PORT))
#define LMT01_ACMP_INPUT_GPIO_PIN         (LMT01_ACMP_INPUT_PIN)
#define LMT01_ACMP_INPUT_BUSALLOC_A_EVEN  (___LMT01_STRINGIFY4(GPIO_ABUSALLOC_AEVEN, LMT01_ACMP_INPUT_BUSALLOC_INDEX, _ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_INPUT_BUSALLOC_A_ODD   (___LMT01_STRINGIFY4(GPIO_ABUSALLOC_AODD, LMT01_ACMP_INPUT_BUSALLOC_INDEX, _ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_INPUT_BUSALLOC_B_EVEN  (___LMT01_STRINGIFY4(GPIO_BBUSALLOC_BEVEN, LMT01_ACMP_INPUT_BUSALLOC_INDEX, _ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_INPUT_BUSALLOC_B_ODD   (___LMT01_STRINGIFY4(GPIO_BBUSALLOC_BODD, LMT01_ACMP_INPUT_BUSALLOC_INDEX, _ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_INPUT_BUSALLOC_CD_EVEN (___LMT01_STRINGIFY4(GPIO_CDBUSALLOC_CDEVEN, LMT01_ACMP_INPUT_BUSALLOC_INDEX, _ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_INPUT_BUSALLOC_CD_ODD  (___LMT01_STRINGIFY4(GPIO_CDBUSALLOC_CDODD, LMT01_ACMP_INPUT_BUSALLOC_INDEX, _ACMP, LMT01_ACMP_DEV_INDEX))
#define LMT01_ACMP_PRS_OUT                (___LMT01_STRINGIFY3(prsSignalACMP, LMT01_ACMP_DEV_INDEX, _OUT))

#define LMT01_COUNTER_TIMER_DEV           (___LMT01_STRINGIFY2(TIMER, LMT01_COUNTER_TIMER_INDEX))
#define LMT01_COUNTER_TIMER_CLOCK         (___LMT01_STRINGIFY2(cmuClock_TIMER, LMT01_COUNTER_TIMER_INDEX))
#define LMT01_COUNTER_TIMER_IRQ           (___LMT01_STRINGIFY3(TIMER, LMT01_COUNTER_TIMER_INDEX, _IRQn))
#define LMT01_COUNTER_TIMER_PRS_CC1       (___LMT01_STRINGIFY3(prsConsumerTIMER, LMT01_COUNTER_TIMER_INDEX, _CC1))

#define LMT01_ONESHOT_TIMER_DEV           (___LMT01_STRINGIFY2(TIMER, LMT01_ONESHOT_TIMER_INDEX))
#define LMT01_ONESHOT_TIMER_CLOCK         (___LMT01_STRINGIFY2(cmuClock_TIMER, LMT01_ONESHOT_TIMER_INDEX))
#define LMT01_ONESHOT_TIMER_IRQ           (___LMT01_STRINGIFY3(TIMER, LMT01_ONESHOT_TIMER_INDEX, _IRQn))
#define LMT01_ONESHOT_TIMER_GPIO_PORT     (___LMT01_STRINGIFY2(gpioPort, LMT01_ONESHOT_TIMER_PORT))
#define LMT01_ONESHOT_TIMER_GPIO_PIN      (LMT01_ONESHOT_TIMER_PIN)

static int lmt01_prs_channel = -1;

static void lmt01_acmp_init();
static void lmt01_counter_timer_init();
static uint32_t lmt01_counter_timer_value();
static void lmt01_oneshot_timer_start();
static bool lmt01_oneshot_timer_active();

void lmt01_init()
{
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

	GPIO_PinModeSet(LMT01_ONESHOT_TIMER_GPIO_PORT, LMT01_ONESHOT_TIMER_GPIO_PIN, gpioModePushPull, 0);

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

	GPIO_PinModeSet(LMT01_ACMP_INPUT_GPIO_PORT, LMT01_ACMP_INPUT_GPIO_PIN, gpioModeInput, 0);

	ACMP_Init_TypeDef acmpInit = ACMP_INIT_DEFAULT;
	ACMP_Init(LMT01_ACMP_DEV, &acmpInit);
	ACMP_ChannelSet(LMT01_ACMP_DEV, ACMP_TRESHOLD, LMT01_ACMP_INPUT);
	if (LMT01_ACMP_INPUT_GPIO_PORT == gpioPortA)
	{
		if(LMT01_ACMP_INPUT_PIN & 1)
		{
			GPIO->ABUSALLOC = LMT01_ACMP_INPUT_BUSALLOC_A_ODD;
		}
		else
		{
			GPIO->ABUSALLOC = LMT01_ACMP_INPUT_BUSALLOC_A_EVEN;
		}
	}
	else if (LMT01_ACMP_INPUT_GPIO_PORT == gpioPortB)
	{
		if(LMT01_ACMP_INPUT_PIN & 1)
		{
			GPIO->BBUSALLOC = LMT01_ACMP_INPUT_BUSALLOC_B_ODD;
		}
		else
		{
			GPIO->BBUSALLOC = LMT01_ACMP_INPUT_BUSALLOC_B_EVEN;
		}
	}
	else if ((LMT01_ACMP_INPUT_GPIO_PORT == gpioPortC) || (LMT01_ACMP_INPUT_GPIO_PORT == gpioPortD))
	{
		if(LMT01_ACMP_INPUT_PIN & 1)
		{
			GPIO->CDBUSALLOC = LMT01_ACMP_INPUT_BUSALLOC_CD_ODD;
		}
		else
		{
			GPIO->CDBUSALLOC = LMT01_ACMP_INPUT_BUSALLOC_CD_EVEN;
		}
	}

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

	GPIO->TIMERROUTE[LMT01_ONESHOT_TIMER_INDEX].CC0ROUTE = (LMT01_ONESHOT_TIMER_GPIO_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
		| (LMT01_ONESHOT_TIMER_GPIO_PIN << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
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

#else // _SILICON_LABS_32B_SERIES_2
#warning "LMT01 driver is not supported on series 1."
void lmt01_init()
{
}

int32_t lmt01_read_temperature()
{
	return -500;
}
#endif // _SILICON_LABS_32B_SERIES_2
