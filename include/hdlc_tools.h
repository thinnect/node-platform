#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct hdlc_encoder {
	const uint8_t* payload;
	uint8_t payload_length;
	const uint8_t* checksum;
	uint8_t checksum_length;
	int16_t ptr;
	uint8_t escape;
} hdlc_encoder_t;

typedef struct hdlc_decoder {
	uint8_t data[160];
	uint8_t length;

	uint8_t escape;
} hdlc_decoder_t;

int hdlc_encode(uint8_t out[], uint8_t outlen, const uint8_t in[], uint8_t inlen);

void hdlc_encoder_init(hdlc_encoder_t* encoder,
                       const uint8_t payload[], uint8_t plen,
                       const uint8_t checksum[], uint8_t clen);
bool hdlc_encoder_next(hdlc_encoder_t* encoder, uint8_t* data);

void hdlc_decoder_init(hdlc_decoder_t* decoder);
int hdlc_decoder_append(hdlc_decoder_t* decoder, uint8_t data);
int hdlc_decoder_copy(hdlc_decoder_t* decoder, uint8_t buffer[], uint8_t length);

#define HDLC_START_FLAG     0x7e
#define HDLC_STOP_FLAG      0x7e
#define HDLC_CONTROL_ESCAPE 0x7d

#define ERROR_HDLC_ENCODER_OVERFLOW -1
