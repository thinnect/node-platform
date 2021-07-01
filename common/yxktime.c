/**
 * Century-epoch time functions.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "yxktime.h"

#include <stddef.h>
#include <stdint.h>

uint32_t time_yxk (const time_t * t)
{
	if((time_t)(-1) != *t)
	{
		struct tm yxk;
	/*	if (NULL != gmtime_r(t, &yxk))
		{
			yxk_zero(&yxk);
			return (uint32_t)(*t - mktime(&yxk));
		}*/
	}
	return UINT32_MAX;
}

time_t yxk_time (uint32_t yxks, const time_t * now)
{
	if (*now != (time_t)(-1))
	{
		struct tm epoch;
	/*	if (NULL != gmtime_r(now, &epoch))
		{
			yxk_zero(&epoch);
			return (time_t)mktime(&epoch) + yxks;
		}*/
	}
	return (time_t)(-1);
}

void yxk_zero (struct tm* yxk)
{
	yxk->tm_sec = 0;
	yxk->tm_min = 0;
	yxk->tm_hour = 0;
	yxk->tm_mday = 1;
	yxk->tm_mon = 0;
	yxk->tm_year = ((yxk->tm_year) / 100) * 100; // Beginning of century, 2000-01-01 00:00:00 = 946684800
	yxk->tm_wday = 0; // Does not really matter
	yxk->tm_yday = 0; // Does not really matter
	yxk->tm_isdst = 0;
}
