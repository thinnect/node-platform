/*
 * SiLabs RETARGET_Serial inspired HDLC UART retargeting solution.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#ifndef SERIAL_HDLC_H_
#define SERIAL_HDLC_H_

#include <stdint.h>

/**
 * You are allowed to send acks from this callback.
 */
typedef void serial_hdlc_receive_f (const uint8_t data[], uint8_t length);

void serial_hdlc_init (serial_hdlc_receive_f * receiver);

void serial_hdlc_enable (void);
void serial_hdlc_disable (void);

int serial_hdlc_send (const uint8_t* out, uint8_t len);

#endif//SERIAL_HDLC_H_
