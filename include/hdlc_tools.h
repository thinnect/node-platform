/**
 * HDLC tools implementation. An encoder and decoder.
 *
 * Copyright Thinnect Inc. 2019
 * @author Raido Pahtma
 * @license MIT
 */
#ifndef HDLC_TOOLS_H_
#define HDLC_TOOLS_H_

#include <stdint.h>
#include <stdbool.h>

#define HDLC_START_FLAG     0x7e
#define HDLC_STOP_FLAG      0x7e
#define HDLC_CONTROL_ESCAPE 0x7d

#define ERROR_HDLC_ENCODER_OVERFLOW -1

typedef struct hdlc_encoder
{
	const uint8_t * payload;
	uint8_t payload_length;
	const uint8_t * checksum;
	uint8_t checksum_length;
	int16_t ptr;
	uint8_t escape;
} hdlc_encoder_t;

typedef struct hdlc_decoder
{
	uint8_t data[160];
	uint8_t length;

	uint8_t escape;
} hdlc_decoder_t;

/**
 * HDLC-encode data from in to out.
 *
 * @param out Output buffer.
 * @param outlen Output buffer.
 * @param in Input buffer.
 * @param inlen Input buffer length.
 * @return Length of encoded data or < 0 for error.
 */
int hdlc_encode (uint8_t out[], uint8_t outlen, const uint8_t in[], uint8_t inlen);

/**
 * HDLC encoder initialization with user-provided buffers.
 *
 * The encoder is able to combine 2 buffers into a single output stream -
 * often the user has a data buffer and then a separately stored CRC that
 * needs to be inserted into the same encoded stream. It is ok, to pass NULL, 0
 * for the second buffer when not needed.
 *
 * @param encoder hdlc_encoder structure to initialize.
 * @param payload First data buffer.
 * @param plen First data buffer length.
 * @param checksum Second data buffer (checksum).
 * @param clen Second data buffer length.
 */
void hdlc_encoder_init (hdlc_encoder_t* encoder,
                        const uint8_t payload[], uint8_t plen,
                        const uint8_t checksum[], uint8_t clen);

/**
 * Get the next symbol for the stream from the HDLC encoder.
 * Keeps storing HDLC_STOP_FLAG and returning false.
 * The data should still be sent when false is returned the first time.
 *
 * @param encoder HDLC encoder.
 * @param data Data storage location.
 * @return True, if still more data in the encoder.
 */
bool hdlc_encoder_next (hdlc_encoder_t* encoder, uint8_t* data);

/**
 * HDLC decoder initialization.
 * The user should initialize a decoder and keep feeding it data with the append
 * function until append returns > 0. Then copy out the data and reset the
 * decider by calling init again.
 *
 * @param decoder Pointer to HDLC decoder to initialize.
 */
void hdlc_decoder_init (hdlc_decoder_t* decoder);

/**
 * Insert data to the HDLC decoder.
 *
 * @param decoder HDLC decoder pointer.
 * @param data Received data byte.
 * @param Length of the received frame, 0 if frame not complete, <0 for error.
 */
int hdlc_decoder_append (hdlc_decoder_t* decoder, uint8_t data);

/**
 * Copy data out of the decoder.
 *
 * @param decoder HDLC decoder pointer.
 * @param buffer Buffer to copy decoded data to.
 * @param length Data buffer length, >= size returned by append.
 */
int hdlc_decoder_copy (hdlc_decoder_t* decoder, uint8_t buffer[], uint8_t length);

#endif//HDLC_TOOLS_H_
