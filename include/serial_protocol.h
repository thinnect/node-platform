#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef bool serial_receive_f (const uint8_t data[], uint8_t length);
typedef void serial_send_done_f (const uint8_t data[], uint8_t length);

void serial_protocol_init (serial_receive_f * receiver);

void serial_protocol_receive (const uint8_t data[], uint8_t length);

/**
 * Send a regular serial packet.
 * @param data - Packet payload.
 * @param length - Packet payload length.
 * @return True, if packet sent.
 */
bool serial_protocol_send (const uint8_t data[], uint8_t length);

/**
 * Send a serial packet and request an acknowledgement.
 * @param data - Packet payload.
 * @param length - Packet payload length.
 * @return < 0 for error, sequence number otherwise (0-255).
 */
int16_t serial_protocol_send_ackpacket (const uint8_t data[], uint8_t length);


void serial_protocol_send_done (uint8_t seq, bool acked);
