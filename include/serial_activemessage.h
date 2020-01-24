/**
* MistComm SerialActiveMessage layer.
*
* Many instances can be created, each needs a serial_protocol.
* Instances will set up a dispatcher with dispatch ID 0x00.
*
* @author Raido Pahtma
* @license MIT
*/
#ifndef SERIAL_ACTIVEMESSAGE_H_
#define SERIAL_ACTIVEMESSAGE_H_

#include "mist_comm_am.h"
#include "mist_comm.h"
#include "serial_protocol.h"

#include "cmsis_os2.h"

// The serial_activemessage instance structure
typedef struct serial_activemessage serial_activemessage_t;

/**
 * Initialize the SerialActiveMessage layer, providing it memory and access
 * to a lower layer SerialProtocol.
 *
 * @param sam - The SerialActiveMessage to initialize, a pointer to a persistent
 *              memory structure.
 * @param spr - Pointer to an initialized SerialProtocol layer.
 *
 * @return a MistComm instance that can be used for sending/receiving messages
 *         or NULL for failure.
 */
comms_layer_t* serial_activemessage_init (serial_activemessage_t * sam,
                                          serial_protocol_t * spr);

/**
 * Deinitialize the SerialActiveMessage layer. The memory used by the instance
 * may be freed after this.
 *
 * @param sam - An initialized SerialActiveMessage
 * @return true if successful, false if busy
 */
bool serial_activemessage_deinit (serial_activemessage_t * sam);


// Internal details to allow memory allocation----------------------------------

// send queue structure
typedef struct sam_queue_element sam_queue_element_t;
struct sam_queue_element
{
	comms_msg_t* msg;
	comms_send_done_f *send_done;
	void *user;
	sam_queue_element_t* next;
};

#ifndef SERIAL_ACTIVEMESSAGE_QUEUE_LENGTH
#define SERIAL_ACTIVEMESSAGE_QUEUE_LENGTH 3
#endif//SERIAL_ACTIVEMESSAGE_QUEUE_LENGTH

#ifndef SERIAL_ACTIVEMESSAGE_MAX_MESSAGE_LENGTH
#define SERIAL_ACTIVEMESSAGE_MAX_MESSAGE_LENGTH 128
#endif//SERIAL_ACTIVEMESSAGE_MAX_MESSAGE_LENGTH

struct serial_activemessage
{
	comms_layer_am_t base;
	serial_dispatcher_t dispatcher;
	serial_protocol_t* protocol;

	osMutexId_t mutex;
	osTimerId_t timer;

	sam_queue_element_t * sending;
	sam_queue_element_t * send_queue;
	sam_queue_element_t * free_queue;
	sam_queue_element_t queue_memory[SERIAL_ACTIVEMESSAGE_QUEUE_LENGTH];

	bool send_busy;
	uint8_t send_buffer[SERIAL_ACTIVEMESSAGE_MAX_MESSAGE_LENGTH];
};

#endif//SERIAL_ACTIVEMESSAGE_H_
