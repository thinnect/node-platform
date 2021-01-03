/**
 * Unix time / network time synchronization functions for Thinnect Mesh.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef BASIC_RTOS_BEATSTACK_TIMESYNC_H_
#define BASIC_RTOS_BEATSTACK_TIMESYNC_H_

#include <stdint.h>

/**
 * Use received network time offset to set unix time.
 * @param offset network time epoch offset (time_since_epoch = osCounterGetSecond() + offset).
 */
void basic_nw_time_changed (uint32_t offset);

/**
 * Get unix time and set network time.
 */
void basic_change_nw_time ();

#endif//BASIC_RTOS_BEATSTACK_TIMESYNC_H_
