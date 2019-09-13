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

/**
 * Receive callback.
 * @param data - Received packet payload.
 * @param length - Received packet payload length.
 * @return True, if packet accepted for processing, False if dropped.
 */
typedef bool serial_receive_f (const uint8_t data[], uint8_t length);

/**
 * SendDone callback.
 * @param data - Sent packet payload.
 * @param length - Sent packet payload length.
 * @param acked - True if ack requested and received.
 */
typedef void serial_send_done_f (const uint8_t data[], uint8_t length, bool acked);

/**
 * Initialize the serial-protocol layer.
 *
 * @param rcvr - Receive callback.
 * @param sdf - SendDone callback function.
 */
void serial_protocol_init (serial_receive_f * rcvr, serial_send_done_f * sdf);

/**
 * Send a serial-protocol packet.
 *
 * @param data - Packet payload.
 * @param length - Packet payload length.
 * @param ack - wait for ack.
 * @return True, if the sdf function provided in init will be called in the future.
 */
bool serial_protocol_send (const uint8_t data[], uint8_t length, bool ack);

/**
 * SerialProtocol layer receive - insert packets received by lower layer.
 *
 * @param data - Packet payload.
 * @param length - Packet payload length.
 */
void serial_protocol_receive (const uint8_t data[], uint8_t length);

#endif//SERIAL_PROTOCOL_H_
