/**
 * A very basic radio duty cycling component.
 *
 * The component periodically wakes up the radio for the specified time by
 * initiating a sleep block, releasing it again after the specified time has
 * passed. Other components may apply their own sleep blocks when needed, this
 * component simply provides periodic opportunities for RX.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef RADIO_DUTY_H_
#define RADIO_DUTY_H_

#include "mist_comm.h"

/**
 * Initialize radio duty cycling according to predefined parameters.
 * @param comms The communications layer that must be duty cycled.
 * @param sleep_ms Duration to sleep.
 * @param awake_ms Duration to stay awake.
 */
void radio_duty_init(comms_layer_t * comms, uint32_t sleep_ms, uint32_t awake_ms);

#endif//RADIO_DUTY_H_
