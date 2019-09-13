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

typedef struct serial_dispatcher serial_dispatcher_t;

/**
 * Receive callback.
 * @param data - Received packet payload.
 * @param length - Received packet payload length.
 * @return True, if packet accepted for processing, False if dropped.
 */
typedef bool serial_receive_f (uint8_t dispatch, const uint8_t data[], uint8_t length);

/**
 * SendDone callback.
 * @param data - Sent packet payload.
 * @param length - Sent packet payload length.
 * @param acked - True if ack requested and received.
 */
typedef void serial_send_done_f (uint8_t dispatch, const uint8_t data[], uint8_t length, bool acked);

/**
 * Initialize the serial-protocol layer.
 *
 * @param dflt_rcvr - Default receive callback, called when no dispatcher registered.
 * @param sdf - SendDone callback function.
 */
void serial_protocol_init (serial_receive_f * dflt_rcvr);

/**
 * Initialize the serial-protocol layer.
 *
 * @param dispatch - SerialProtocol identifier, the dispatch byte.
 * @param disp - Dispatcher datastructure.
 * @param rcvr - Receive callback function.
 * @param sdf - SendDone callback function.
 * @return true, if added, false if something wrong.
 */
bool serial_protocol_add_dispatcher(uint8_t dispatch,
                                    serial_dispatcher_t * disp,
                                    serial_receive_f * rcvr,
                                    serial_send_done_f * sdf);

/**
 * Remove a dispatcher.
 *
 * @param dispatcher - The dispatcher to remove.
 * @return true, if removed, false if not found.
 */
bool serial_protocol_remove_dispatcher(serial_dispatcher_t * dispatcher);

/**
 * Send a serial-protocol packet.
 *
 * @param data - Packet payload.
 * @param length - Packet payload length.
 * @param ack - wait for ack.
 * @return True, if the sdf function provided in init will be called in the future.
 */
bool serial_protocol_send (serial_dispatcher_t * dispatcher,
                           const uint8_t data[], uint8_t length,
                           bool ack);

/**
 * SerialProtocol layer receive - packets from the lower layer would be inserted here.
 *
 * @param data - Packet payload.
 * @param length - Packet payload length.
 */
void serial_protocol_receive (const uint8_t data[], uint8_t length);


// Dispatcher definition for storage size calculation
struct serial_dispatcher
{
    uint8_t dispatch;

    const uint8_t * data;
    uint8_t data_length;
    bool ack;

    serial_receive_f * freceiver;
    serial_send_done_f * fsenddone;

    serial_dispatcher_t * next;
};

#endif//SERIAL_PROTOCOL_H_
