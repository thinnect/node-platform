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

#define RAIL_CB_FLAG        1
#define SEND_FLAG           2
#define FAIL_FLAG           4
#define ACK_WAIT_FLAG       8
#define RESEND_FLAG         16
#define RAIL_SEND_DONE      32
#define RAIL_SEND_BUSY      64
#define RAIL_SEND_FAIL      128
#define RAIL_RX_BUSY        256
#define RAIL_RX_SUCCESS     512
#define RAIL_RX_OVERFLOW    1024
#define RAIL_RX_FRAME_ERROR 2048
#define RAIL_RX_ABORT       4096
#define RAIL_RX_FAIL        8192
#define RAIL_TXACK_SENT     16384
#define RAIL_RXACK_TIMEOUT  32768

extern comms_layer_am_t radio_iface;
extern uint16_t radio_address;

comms_layer_t* radio_init(uint16_t channel, uint16_t pan_id, uint16_t address);
void radio_idle();
void radio_reenable();
void radio_reenable_channel_panid(uint16_t channel, uint16_t pan_id);

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
