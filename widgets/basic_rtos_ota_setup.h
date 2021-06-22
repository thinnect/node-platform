/**
 * A supposedly common and basic OTA on rtos and multi-hop setup procedure.
 *
 * !!! Look at the code, make sure your use-case falls under the same
 *     understanding of basic setup. !!!
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef BASIC_RTOS_OTA_SETUP_H_
#define BASIC_RTOS_OTA_SETUP_H_

#include "mist_comm.h"

/**
 * Configure OTA updater.
 *
 * @param p_beat_comm     - Pointer to multi-hop radio.
 * @param p_direct_comm   - Pointer to radio under the multi-hop radio.
 * @param sleep_control   - Sleep control should be used to control direct radio. // TODO probably should get rid of it
 * @param p_feed_watchdog - Acces to watchdog management, global watchdog feeding
 *                          will stop if set to false. // TODO remove once a better health notification is available.
 */
void basic_rtos_ota_setup (comms_layer_t * p_beat_comm, comms_layer_t * p_direct_comm, bool sleep_control, bool * p_feed_watchdog);

#endif//BASIC_RTOS_OTA_SETUP_H_
