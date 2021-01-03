/**
 * Century-epoch time functions.
 * Conversions to Unix time require knowledge of current century.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef YXKTIME_H_
#define YXKTIME_H_

#include <time.h>

/**
 * Convert the Unix time to century time.
 * @param t A Unix timestamp.
 * @return Seconds since the start of the century.
 */
uint32_t time_yxk (const time_t * t);

/**
 * Convert the century time to Unix time.
 * @param yxks Timestamp to convert.
 * @param now  A Unix timestamp from the current century.
 * @return Unix timestamp.
 */
time_t yxk_time (uint32_t yxks, const time_t * now);

/**
 * Reset the struct tm to the beginning of the century.
 * @param yxktm Structure to reset to the beginning of the century.
 */
void yxk_zero (struct tm * yxktm);

#endif//YXKTIME_H_
