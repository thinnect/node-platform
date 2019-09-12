#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef void serial_receive_f (const uint8_t data[], uint8_t length);

void serial_protocol_init (serial_receive_f * receiver);

void serial_protocol_receive (const uint8_t data[], uint8_t length);

bool serial_protocol_send (const uint8_t data[], uint8_t length, bool ack);
void serial_protocol_send_done (bool acked);
