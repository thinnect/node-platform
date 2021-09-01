/**
 * Unix time / network time synchronization functions for Thinnect Mesh.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "basic_rtos_beatstack_timesync.h"

#include "time_rtc.h"
#include "yxktime.h"

#include "cmsis_os2_ext.h"

#include "beatstack.h"

#include <inttypes.h>

#include "loglevels.h"
#define __MODUUL__ "ntime"
#define __LOG_LEVEL__ (LOG_LEVEL_basic_rtos_beatstack_timesync & BASE_LOG_LEVEL)
#include "log.h"

void basic_nw_time_changed (uint32_t offset)
{
	uint32_t yxk = osCounterGetSecond() + offset;
	time_t rtc = time(NULL);
	debug1("NW:%"PRIu32 "yxk:%"PRIu32, offset, yxk);
	if((time_t)(-1) == rtc)
	{
		debug1("first"); // Clock totally off, assume it's still the century when the image was built
		rtc = IDENT_TIMESTAMP;
	}

	rtc = yxk_time(yxk, &rtc);
	if ((time_t)(-1) != rtc)
	{
		info1("Time:%"PRIi64, (int64_t)rtc);
		time_rtc_stime(&rtc);
	}
	else err1("rvrs");
}

void basic_change_nw_time ()
{
	uint32_t yxko = 0;
	time_t t = time(NULL);
	if((time_t)(-1) != t)
	{
		yxko = time_yxk(&t) - osCounterGetSecond();
	}
	info1("Time:%"PRIi64", NW:%"PRIu32, (int64_t)t, yxko);

	beatstack_nw_time_set(yxko);
}
