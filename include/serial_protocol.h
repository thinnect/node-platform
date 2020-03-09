/**
 * SerialProtocol layer API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#ifndef SERIAL_PROTOCOL_H_
#define SERIAL_PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#define SERIAL_PROTOCOL_ACK_TIMEOUT_MS 100UL

typedef struct serial_protocol serial_protocol_t;
typedef struct serial_dispatcher serial_dispatcher_t;

/**
 * Receive callback.
 *
 * @param dispatch The dispatch ID of the received data.
 * @param data     Received packet payload.
 * @param length   Received packet payload length.
 * @param user     The user pointer set with add_dispatcher.
 *
 * @return True, if packet accepted for processing, False if dropped.
 */
typedef bool serial_receive_f (uint8_t dispatch, const uint8_t data[], uint8_t length, void * user);

/**
 * SendDone callback.
 *
 * @param dispatch The dispatch ID of the sent data.
 * @param data     Sent packet payload.
 * @param length   Sent packet payload length.
 * @param acked    True if ack requested and received.
 * @param user     The user pointer set with add_dispatcher.
 */
typedef void serial_send_done_f (uint8_t dispatch, const uint8_t data[], uint8_t length, bool acked, void * user);

/**
 * The lower layer send function.
 * @param data   Data to send.
 * @param length Length of the data.
 *
 * @param return 0 for success.
 */
typedef int raw_serial_send_f (const uint8_t data[], uint8_t length);

/**
 * Initialize the serial-protocol layer.
 *
 * @param sp        The SerialProtocol to initialize.
 * @param sendf     The send function to use.
 * @param dflt_rcvr Default receive callback, called when no dispatcher registered.
 *
 * @return true for success, false for failure
 */
bool serial_protocol_init (serial_protocol_t * sp,
                           raw_serial_send_f * sendf,
                           serial_receive_f * dflt_rcvr);

/**
 * Initialize the serial-protocol layer.
 *
 * @param dispatch SerialProtocol identifier, the dispatch byte.
 * @param disp     Dispatcher datastructure.
 * @param rcvr     Receive callback function.
 * @param sdf      SendDone callback function.
 * @param user     User pointer passed to callbacks.
 *
 * @return true, if added, false if something wrong.
 */
bool serial_protocol_add_dispatcher(serial_protocol_t * sp,
                                    uint8_t dispatch,
                                    serial_dispatcher_t * disp,
                                    serial_receive_f * rcvr,
                                    serial_send_done_f * sdf,
                                    void * user);

/**
 * Remove a dispatcher.
 *
 * @param dispatcher The dispatcher to remove.
 *
 * @return true, if removed, false if not found.
 */
bool serial_protocol_remove_dispatcher(serial_protocol_t * sp,
                                       serial_dispatcher_t * dispatcher);

/**
 * Send a serial-protocol packet.
 *
 * @param data   Packet payload.
 * @param length Packet payload length.
 * @param ack    Wait for ack.
 *
 * @return True, if the sdf function provided in init will be called in the future.
 */
bool serial_protocol_send (serial_dispatcher_t * dispatcher,
                           const uint8_t data[], uint8_t length,
                           bool ack);

/**
 * SerialProtocol layer receive.
 * P packets from the lower layer would be inserted here.
 *
 * @param data   Packet payload.
 * @param length Packet payload length.
 */
void serial_protocol_receive (serial_protocol_t * sp,
                              const uint8_t data[], uint8_t length);

/**
 * SerialProtocol layer receive void * version.
 * P packets from the lower layer would be inserted here and then passed to
 * the regular serial_protocol_receive.
 *
 * @param data   Packet payload.
 * @param length Packet payload length.
 */
void serial_protocol_receive_generic (void * sp,
                                      const uint8_t data[], uint8_t length);


/* ---------------------------------------------------------------------------*/


// Dispatcher definition for storage size calculation --------------------------
struct serial_dispatcher
{
    uint8_t dispatch;

    const uint8_t * data;
    uint8_t data_length;
    bool ack;
    bool acked;
    uint32_t send_time; // Time message was sent, used to determine ack timeout

    serial_receive_f * freceiver;
    serial_send_done_f * fsenddone;
    void * user;

    serial_protocol_t * protocol; // parent

    serial_dispatcher_t * next;
};

struct serial_protocol
{
    uint8_t tx_seq_num;
    uint8_t rx_seq_num;

    osMutexId_t mutex;
    osThreadId_t thread;
    osTimerId_t timeout_timer;

    serial_dispatcher_t * p_active_dispatcher;
    serial_dispatcher_t * p_dispatchers;

    serial_receive_f * f_default_receiver;

    raw_serial_send_f * sendf; // Send data using this function

    uint8_t send_buffer[128]; // raw repr of packet being sent
};

#endif//SERIAL_PROTOCOL_H_
