/**
 * Simple Unix time RTC module API.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef TIME_RTC_H_
#define TIME_RTC_H_

#include <time.h>

/**
 * Initialize the RTC system. Call it once after kernel has started.
 */
void time_rtc_init ();

/**
 * Set the current time.
 * @param t The time to set the RTC to.
 * @return 0 for success.
 */
int time_rtc_stime (const time_t * t);

#endif//TIME_RTC_H_
