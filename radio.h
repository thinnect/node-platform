#pragma once

#include <stdint.h>
#include "mist_comm_iface.h"
#include "mist_comm_am.h"

//#define RADIO_CALLBACK_COUNT 8

extern comms_layer_am_t radio_iface;
extern uint16_t radio_address;

//struct radio_callback_struct{
//	uint8_t am_id;
//	void *user;
//	void (*recv)(void *user, const void *buf, int count, uint16_t src, uint16_t dst, uint8_t am_group, uint8_t am_id, int rssi);
//};

//typedef struct radio_callback_struct radio_callback_t;

comms_layer_t* radio_init(uint16_t channel, uint16_t pan_id, uint16_t address);
void radio_idle();
void radio_reenable();
void radio_reenable_channel_panid(uint16_t channel, uint16_t pan_id);
void radio_poll(void);
// int radio_recv_register(uint8_t am_id, void *user, void (*recv)(void *user, const void *buf, int count, uint16_t src, uint16_t dst, uint8_t am_group, uint8_t am_id, int rssi));
// int radio_send(const void *buf, int count, uint16_t dst, uint8_t am_group, uint8_t am_id);

// queue -----------------------------------------------------------------------
typedef struct radio_queue_element radio_queue_element_t;
struct radio_queue_element {
	comms_msg_t* msg;
	comms_send_done_f *send_done;
	void *user;
	radio_queue_element_t* next;
};
