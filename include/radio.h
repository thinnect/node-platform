/*
 * Mist-comm compatible radio API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Raido Pahtma
 */
#pragma once

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

extern comms_layer_am_t radio_iface;
extern uint16_t radio_address;

comms_layer_t* radio_init(uint16_t channel, uint16_t pan_id, uint16_t address);
void radio_idle();
void radio_reenable();
void radio_reenable_channel_panid(uint16_t channel, uint16_t pan_id);

/**
 * Radio statistics - count time radio is initialized, but sleeping.
 * @return sleep time in milliseconds.
 */
uint32_t radio_sleep_time();

/**
 * Poll the radio.
 * @return true if it is doing something.
 */
bool radio_poll(void);

// queue -----------------------------------------------------------------------
typedef struct radio_queue_element radio_queue_element_t;
struct radio_queue_element {
	comms_msg_t* msg;
	comms_send_done_f *send_done;
	void *user;
	radio_queue_element_t* next;
};
