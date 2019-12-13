/*
 * HDLC UART API. A layer implementing this API does HDLC framing and CRC
 * checks using the XMODEM(ITU-T) 16-bit CRC.
 *
 * NOTE: UART configuration is platform dependant.
 *
 * Future versions might make the actual UART configurable in init.
 * Future versions might make CRC configurable in init.
 * Future versions might make HDLC parameters configurable in init.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#ifndef SERIAL_HDLC_H_
#define SERIAL_HDLC_H_

#include <stdint.h>

/**
 * HDLC receive callback, set with init.
 *
 * Data received here has had escape characters removed and CRC checked.
 *
 * You are allowed to send (acks) from this callback.
 *
 * @param rcvr Receiver pointer, set during init.
 * @param data received data, escapes already done.
 * @param length received data length.
 */
typedef void serial_hdlc_receive_f (void * rcvr, const uint8_t data[], uint8_t length);

/**
 * Initialize the HDLC layer, provide a callback pointer.
 *
 * @param rcvf Receiver callback function.
 * @param rcvr Receiver object, passed to callback.
 */
void serial_hdlc_init (serial_hdlc_receive_f * rcvf, void * rcvr);

/**
 * Enable an already initialized HDLC layer.
 */
void serial_hdlc_enable (void);

/**
 * Disable an already initialized HDLC layer.
 */
void serial_hdlc_disable (void);

/**
 * Send some data - it will be framed and a CRC will be added.
 *
 * @param out Data to send.
 * @param len Length of the data.
 * @return 0 for success.
 */
int serial_hdlc_send (const uint8_t* out, uint8_t len);

#endif//SERIAL_HDLC_H_
