/**
 * Simple Unix time RTC module. Records a known reference point between
 * kernel localtime and Unix time and uses that to calculate the time.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "cmsis_os2_ext.h"

#include <time.h>

static osMutexId_t m_time_mutex;
static time_t m_time = (time_t)(-1);
static uint32_t m_local = 0;


void time_rtc_init()
{
	m_time_mutex = osMutexNew(NULL);
}

/**
 * Provide the implementation of time, which is normally weakly defined.
 */
time_t time (time_t * tr)
{
	time_t t = (time_t)(-1);
	while(osOK != osMutexAcquire(m_time_mutex, osWaitForever));
	if (m_time > (time_t)(-1))
	{
		t = m_time + (osCounterGetSecond() - m_local);
	}
	osMutexRelease(m_time_mutex);
	return t;
}

/*
void time64_changed (time_t told, time_t tnew) __attribute__((weak));
void time64_changed (time_t told, time_t tnew)
{
	// Do nothing
}
*/

/**
 * Essentially Linux stime.
 */
int time_rtc_stime (const time_t * t)
{
	while(osOK != osMutexAcquire(m_time_mutex, osWaitForever));
	m_local = osCounterGetSecond();
	m_time = *t;
	osMutexRelease(m_time_mutex);
	return 0;
}
