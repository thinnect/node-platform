/*
 * Mist-comm compatible radio API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Raido Pahtma
 */
#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>
#include "mist_comm_iface.h"
#include "mist_comm_am.h"

// Specify default RFPower set during radio_init. dBm, can use float values.
// For TX output +17dBm, CFLAGS += -DDEFAULT_RFPOWER_DBM=17
// For a really weak -3.2dBm, CFLAGS += -DDEFAULT_RFPOWER_DBM=-3.2
#ifndef DEFAULT_RFPOWER_DBM
#warning "Using DEFAULT_RFPOWER_DBM = 0"
#define DEFAULT_RFPOWER_DBM 0
#endif//DEFAULT_RFPOWER_DBM

/**
 * Initialize the radio.
 *
 * @param channel Initial channel to use.
 * @param pan_id Initial pan_id to use.
 * @param address The 16-bit radio address.
 *
 * @return A pointer to a comms_layer_t radio interface or NULL on failure.
 */
comms_layer_t * radio_init(uint16_t channel, uint16_t pan_id, uint16_t address);

/**
 * Deinitialize the radio. Make sure it is stopped before attempting this!
 * Will block until deinitialized.
 * @param iface A comms_layer_t previously obtained with radio_init.
 */
void radio_deinit (comms_layer_t * iface);

// Configure promiscuous mode
void radio_set_promiscuous(bool promiscuous);

/**
 * Force radio to idle. Only supported on "basic" implementations.
 */
void radio_idle();

/**
 * Pull radio out of idle.
 * Only supported on "basic" implementations.
 */
void radio_reenable();

/**
 * Pull radio out of idle with new parameters.
 * Only supported on "basic" implementations.
 */
void radio_reenable_channel_panid(uint16_t channel, uint16_t pan_id);

/**
 * Radio statistics - count time radio is initialized, but sleeping.
 * @return sleep time in milliseconds.
 */
uint32_t radio_sleep_time();

/**
 * Radio statistics - count number of transmitted packets (including retries).
 * @return number of transmitted packets.
 */
uint32_t radio_tx_packets();

/**
 * Radio statistics - roughly estimate the number of transmitted bytes.
 * @return Transmitted bytes.
 */
uint32_t radio_tx_bytes();

/**
 * Poll the radio.
 * Necessary on "basic" implementations.
 *
 * @return true if it is doing something (outgoing messages queued).
 */
bool radio_poll(void);

#endif//RADIO_H_
