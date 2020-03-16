/**
* MistComm SerialActiveMessage layer implementation.
*
* @author Raido Pahtma
* @license MIT
*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>

#include "cmsis_os2.h"

#include "endianness.h"

#include "mist_comm.h"
#include "serial_protocol.h"
#include "serial_activemessage.h"

#include "loglevels.h"
#define __MODUUL__ "sam"
#define __LOG_LEVEL__ (LOG_LEVEL_serial_activemessage & BASE_LOG_LEVEL)
#include "log.h"

#pragma pack(push, 1)
typedef struct tos_serial_message
{
	uint16_t destination;
	uint16_t source;
	uint8_t payload_length; // payload[] length
	uint8_t group;
	uint8_t amid;
	uint8_t payload[];
	// footer with lqi/rssi
} tos_serial_message_t;
#pragma pack(pop)

static comms_error_t serial_activemessage_send (comms_layer_iface_t * iface,
                                                comms_msg_t * msg,
                                                comms_send_done_f * send_done, void * user);
static bool serial_am_receive(uint8_t dspch, const uint8_t data[], uint8_t length, void* user);
static void serial_am_senddone(uint8_t dspch, const uint8_t data[], uint8_t length, bool acked, void* user);
static void serial_am_timer_cb(void * argument);

comms_layer_t* serial_activemessage_init (serial_activemessage_t* sam, serial_protocol_t * spr)
{
	sam->mutex = osMutexNew(NULL);
	sam->timer = osTimerNew(&serial_am_timer_cb, osTimerOnce, sam, NULL);

	// Initialize send queueing system
	sam->send_busy = false;
	sam->sending = NULL;
	sam->send_queue = NULL;
	sam->free_queue = &(sam->queue_memory[0]);
	sam->free_queue->next = NULL;
	for (uint8_t i=1; i<sizeof(sam->queue_memory)/sizeof(sam_queue_element_t); i++)
	{
		sam->queue_memory[i].next = sam->free_queue;
		sam->free_queue = &(sam->queue_memory[i]);
	}

	// Set up dispatcher
	sam->protocol = spr;
	serial_protocol_add_dispatcher(sam->protocol, 0x00,
		                           &(sam->dispatcher),
		                           &serial_am_receive, &serial_am_senddone,
		                           sam);

	// Set up the mist-comm layer
	// TODO should we be passing 0 as default address?
	// TODO start-stop handlers
	comms_am_create((comms_layer_t *)sam, 0, &serial_activemessage_send, NULL, NULL);

	// serial_activemessage_t is a valid comms_layer_t
	return (comms_layer_t *)sam;
}

bool serial_activemessage_deinit (serial_activemessage_t* sam)
{
	while (osOK != osMutexAcquire(sam->mutex, osWaitForever));

	if ((1 == osTimerIsRunning(sam->timer))
	  ||(NULL != sam->sending))
	{
		osMutexRelease(sam->mutex);
		return false;
	}

	serial_protocol_remove_dispatcher(sam->protocol, &(sam->dispatcher));
	osTimerDelete(sam->timer);

	osMutexRelease(sam->mutex);
	osMutexDelete(sam->mutex);

	return true;
}

static bool serial_am_receive(uint8_t dispatch, const uint8_t data[], uint8_t length, void * user)
{
	serial_activemessage_t * sam = (serial_activemessage_t*)user;
	comms_layer_t * lyr = (comms_layer_t*)sam;
	debugb1("%p data", data, length, sam);

	if (sizeof(tos_serial_message_t) > length)
	{
		warn1("short"); // Message is too short
		return false;
	}

	tos_serial_message_t * m = (tos_serial_message_t*)data;
	if(length - sizeof(tos_serial_message_t) < m->payload_length)
	{
		warn1("p len big"); // Payload is listed as larger than available data
		return false;
	}

	while (osOK != osMutexAcquire(sam->mutex, osWaitForever));

	comms_msg_t msg;
	comms_init_message(lyr, &msg);

	uint8_t* payload = comms_get_payload(lyr, &msg, m->payload_length);
	if (NULL == payload)
	{
		osMutexRelease(sam->mutex);
		err1("pl %d", m->payload_length);
		return false;
	}

	comms_set_packet_type(lyr, &msg, m->amid);
	comms_set_payload_length(lyr, &msg, m->payload_length);
	memcpy(payload, (const void *)m->payload, m->payload_length);

	// comms_set_timestamp(lyr, &msg, timestamp); // TODO Set to now? Make the lowest layer register frame starts ...?

	comms_am_set_destination(lyr, &msg, ntoh16(m->destination));
	comms_am_set_source(lyr, &msg, ntoh16(m->source));

	 // TODO DEFAULT_PAN_ID variable
	debugb1("rx {%02X}%04"PRIX16"->%04"PRIX16"[%02X]",
		payload, comms_get_payload_length(lyr, &msg),
		DEFAULT_PAN_ID,
		comms_am_get_source(lyr, &msg),
		comms_am_get_destination(lyr, &msg),
		comms_get_packet_type(lyr, &msg));

	comms_deliver(lyr, &msg);

    osMutexRelease(sam->mutex);

	return true;
}

/*
* Convert a comms message to a serial-protocol message buffer
* @param buffer - the buffer to store the message
* @param length - the length of the buffer
* @param msg - the message to convert
* @param lyr - the layer to use for interpreting the message
* @return length of the prepared message or 0 for errors
*/
static uint16_t prepare_sp_message(uint8_t buffer[], uint16_t length, comms_msg_t * msg, comms_layer_t * lyr)
{
	uint8_t plen = comms_get_payload_length(lyr, msg);

	if(plen + sizeof(tos_serial_message_t) <= length)
	{
		void * payload = comms_get_payload(lyr, msg, plen);
		if (NULL != payload)
		{
			tos_serial_message_t* m = (tos_serial_message_t*)buffer;

			m->destination = hton16(comms_am_get_destination(lyr, msg));
			m->source = hton16(comms_am_get_source(lyr, msg));
			m->group = DEFAULT_PAN_ID; // TODO variable
			m->amid = comms_get_packet_type(lyr, msg);
			m->payload_length = plen;
			memcpy(m->payload, payload, plen);

			// TODO check if LQI/RSSI is set in the packet
			m->payload[plen] = comms_get_lqi(lyr, msg);
			m->payload[plen+1] = comms_get_rssi(lyr, msg);

			return sizeof(tos_serial_message_t) + plen + 2; // + 2 for LQI/RSSI
		}
		else // Should not happen, unless platform configured incorrectly
		{
			err1("null pl");
		}
	}
	else
	{
		err1("pl size %d", (unsigned int)plen);
	}
	return 0;
}

static void serial_am_senddone(uint8_t dispatch, const uint8_t data[], uint8_t length, bool acked, void * user)
{
	serial_activemessage_t * sam = (serial_activemessage_t*)user;
	comms_layer_t * lyr = (comms_layer_t*)sam;

	debugb1("snt(a:%d) %02X", data, length, (int)acked, (unsigned int)dispatch);

	// Set flags on the message that was just sent
	//comms_set_timestamp(lyr, sam->sending->msg, now());
	if (acked)
	{
		_comms_set_ack_received(lyr, sam->sending->msg);
	}

	// Signal send done events
	sam->sending->send_done(lyr, sam->sending->msg, COMMS_SUCCESS, sam->sending->user);
	sam->sending->msg = NULL;

	// Mark sent and defer to send other pending messages
	while (osOK != osMutexAcquire(sam->mutex, osWaitForever));
	sam->send_busy = false;
	osMutexRelease(sam->mutex);
	osTimerStart(sam->timer, 1UL);
}

static void serial_am_timer_cb(void * argument)
{
	serial_activemessage_t * sam = (serial_activemessage_t*)argument;
	comms_layer_t * lyr = (comms_layer_t*)sam;

	while (osOK != osMutexAcquire(sam->mutex, osWaitForever));
	if(sam->send_busy)
	{
		debug1("bsy"); // Must wait for last send to complete
		osMutexRelease(sam->mutex);
		return;
	}

	if (NULL != sam->sending)
	{
		// Return the queue element to the free queue
		sam->sending->next = sam->free_queue;
		sam->free_queue = sam->sending;
		sam->sending = NULL;
	}

	if (NULL != sam->send_queue)
	{
		sam->sending = sam->send_queue;
		sam->send_queue = sam->send_queue->next;
		sam->send_busy = true;
	}
	osMutexRelease(sam->mutex);

	if (NULL != sam->sending)
	{
		uint16_t length = prepare_sp_message(sam->send_buffer, sizeof(sam->send_buffer), sam->sending->msg, lyr);
		if(length > 0)
		{
			if(true == serial_protocol_send(&(sam->dispatcher), sam->send_buffer, length, false))
			{
				debug1("snd %p %p %p", sam->sending, sam->send_queue, sam->free_queue);
				return;
			}
			else
			{
				err1("busy?");
			}
		}
		else
		{
			err1("prep");
		}

		// Something bad has happened, return message to user with error
		sam->sending->send_done(lyr, sam->sending->msg, COMMS_EINVAL, sam->sending->user);
		sam->sending->msg = NULL; // Pointer has been returned

		while (osOK != osMutexAcquire(sam->mutex, osWaitForever));
		sam->send_busy = false;
		osMutexRelease(sam->mutex);
		osTimerStart(sam->timer, 1L); // Try next message
	}
	else
	{
		debug1("idle");
	}
}

static comms_error_t serial_activemessage_send (comms_layer_iface_t * iface,
                                                comms_msg_t * msg,
                                                comms_send_done_f * send_done, void * user)
{
	comms_error_t result = COMMS_SUCCESS;
	serial_activemessage_t * sam = (serial_activemessage_t*)iface;
	while (osOK != osMutexAcquire(sam->mutex, osWaitForever));

	debug1("snd %p %p", sam->send_queue, sam->free_queue);
	if (NULL != sam->free_queue)
	{
		sam_queue_element_t ** qe = &(sam->send_queue);
		while (NULL != *qe)
		{
			qe = &((*qe)->next);
		}

		*qe = sam->free_queue;
		sam->free_queue = sam->free_queue->next;

		(*qe)->msg = msg;
		(*qe)->send_done = send_done;
		(*qe)->user = user;
		(*qe)->next = NULL;

		debug1("q %p %p", sam->send_queue, sam->free_queue);
		osTimerStart(sam->timer, 1UL);
	}
	else // Queue full
	{
		result = COMMS_EBUSY;
	}

	osMutexRelease(sam->mutex);

	return result;
}
