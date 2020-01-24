/**
 * HDLC tools implementation.
 *
 * Copyright Thinnect Inc. 2019
 * @author Raido Pahtma
 * @license MIT
 */
#include <string.h>

#include "hdlc_tools.h"

int hdlc_encode(uint8_t out[], uint8_t outlen, const uint8_t in[], uint8_t inlen)
{
	uint8_t o = 0;

	if(outlen < 2) // at least START+STOP
	{
		return ERROR_HDLC_ENCODER_OVERFLOW;
	}

	out[o++] = HDLC_START_FLAG;

	for(uint8_t i=0;i<inlen;i++)
	{
		uint8_t data = in[i];
		if(o >= outlen)
		{
			return -1;
		}
		if((data == HDLC_CONTROL_ESCAPE)
		 ||(data == HDLC_START_FLAG)
		 ||(data == HDLC_STOP_FLAG))
		{
			out[o++] = HDLC_CONTROL_ESCAPE;
			data ^= 0x20;
			if(o >= outlen)
			{
				return -1;
			}
		}
		out[o++] = data;
	}

	if(o >= outlen)
	{
		return -1;
	}
	out[o++] = HDLC_STOP_FLAG;
	return o;
}

void hdlc_encoder_init(hdlc_encoder_t* encoder,
                       const uint8_t payload[], uint8_t plen,
                       const uint8_t checksum[], uint8_t clen)
{
	encoder->payload = payload;
	encoder->payload_length = plen;
	encoder->checksum = checksum;
	encoder->checksum_length = clen;
	encoder->ptr = -1;
	encoder->escape = false;
}

bool hdlc_encoder_next(hdlc_encoder_t* encoder, uint8_t* data)
{
	if(encoder->ptr < 0)
	{
		encoder->ptr = 0;
		*data = HDLC_START_FLAG;
		return true;
	}

	if(encoder->ptr == encoder->payload_length + encoder->checksum_length)
	{
		encoder->ptr++;
		*data = HDLC_STOP_FLAG;
		return true;
	}

	if(encoder->ptr > encoder->payload_length + encoder->checksum_length)
	{
		*data = HDLC_STOP_FLAG;
		return false;
	}

	if(encoder->ptr < encoder->payload_length)
	{
		*data = encoder->payload[encoder->ptr];
	}
	else
	{
		*data = encoder->checksum[encoder->ptr - encoder->payload_length];
	}

	if(encoder->escape)
	{
		encoder->escape = false;
		*data ^= 0x20;
		encoder->ptr++;
	}
	else
	{
		if((*data == HDLC_CONTROL_ESCAPE)
		 ||(*data == HDLC_START_FLAG)
		 ||(*data == HDLC_STOP_FLAG))
		{
		 	encoder->escape = true;
		 	*data = HDLC_CONTROL_ESCAPE;
		}
		else
		{
			encoder->ptr++;
		}
	}
	return true;
}

void hdlc_decoder_init(hdlc_decoder_t* decoder)
{
	decoder->length = 0;
	decoder->escape = false;
}

int hdlc_decoder_append(hdlc_decoder_t* decoder, uint8_t data)
{
	if(data == HDLC_STOP_FLAG)
	{
		return decoder->length;
	}

	if(data == HDLC_CONTROL_ESCAPE)
	{
		decoder->escape = true;
		return 0;
	}

	if(decoder->escape)
	{
		decoder->escape = false;
		data = data ^ 0x20;
	}

	if(decoder->length < sizeof(decoder->data))
	{
		decoder->data[decoder->length++] = data;
		return 0;
	}
	return -1;
}

int hdlc_decoder_copy(hdlc_decoder_t* decoder, uint8_t buffer[], uint8_t length)
{
	if(length >= decoder->length)
	{
		memcpy(buffer, decoder->data, decoder->length);
		return decoder->length;
	}
	return -1;
}
