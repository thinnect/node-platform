#include "unity.h"
#include "string.h"
#include "hdlc_tools.h"

void setUp(void)
{
}

void tearDown(void)
{
}

void test_HdlcEncodeZeroLength()
{
	hdlc_encoder_t encoder;
	uint8_t outbuf[255];
	uint8_t o;

	uint8_t ibuf1[] = {0x0};
	uint8_t ebuf1[] = {0x7E, 0x7E};

	hdlc_encoder_init(&encoder, ibuf1, 0, NULL, 0);
	for (o = 0; o < sizeof(outbuf); o++)
	{
		if (false == hdlc_encoder_next(&encoder, &(outbuf[o])))
		{
			break;
		}
	}

	TEST_ASSERT_EQUAL_UINT8(sizeof(ebuf1), o);

	TEST_ASSERT_EQUAL_UINT8_ARRAY(ebuf1, outbuf, sizeof(ebuf1));
}


void encodeHelper(uint8_t in[], uint8_t ilen, uint8_t out[], uint8_t olen)
{
	hdlc_encoder_t encoder;
	uint8_t outbuf[255];
	uint8_t o;

	// Test encoder encode -------------------------------------------------
	hdlc_encoder_init(&encoder, in, ilen, NULL, 0);

	memset(outbuf, 0, sizeof(outbuf));
	for (o = 0; o < sizeof(outbuf); o++)
	{
		if (false == hdlc_encoder_next(&encoder, &(outbuf[o])))
		{
			break;
		}
	}

	TEST_ASSERT_EQUAL_UINT8(olen, o);

	TEST_ASSERT_EQUAL_UINT8_ARRAY(out, outbuf, olen);


	// Test direct encode --------------------------------------------------
	memset(outbuf, 0, sizeof(outbuf));

	int outlen = hdlc_encode(outbuf, sizeof(outbuf), in, ilen);

	TEST_ASSERT_EQUAL_UINT8(olen, outlen);

	TEST_ASSERT_EQUAL_UINT8_ARRAY(out, outbuf, olen);
}


void test_HdlcEncodeSingleSymbol()
{
	for(uint16_t i=0; i<= 0xFF; i++)
	{
		uint8_t len;
		uint8_t in[] = {i};
		uint8_t out[4];

		switch(i)
		{
			case 0x7D: // Escape character
			case 0x7E: // Framing character
				out[0] = 0x7E;
				out[1] = 0x7D;
				out[2] = 0x20 ^ i;
				out[3] = 0x7E;
				len = 4;
			break;
			default:
				out[0] = 0x7E;
				out[1] = i;
				out[2] = 0x7E;
				len = 3;
			break;
		}

		encodeHelper(in, sizeof(in), out, len);
	}
}


void test_HdlcEncodeSimpleSample()
{
	uint8_t i[] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
	uint8_t o[] = {0x7E, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x7E};

	encodeHelper(i, sizeof(i), o, sizeof(o));
}


void test_HdlcEncodeEscapeSample()
{
	uint8_t i[] = {            0x7E,       0x7E,       0x7D,       0x7D,       0x7E,       0x7E};
	uint8_t o[] = {0x7E, 0x7D, 0x5E, 0x7D, 0x5E, 0x7D, 0x5D, 0x7D, 0x5D, 0x7D, 0x5E, 0x7D, 0x5E, 0x7E};

	encodeHelper(i, sizeof(i), o, sizeof(o));
}


void test_HdlcDecodeSingeSymbol()
{
	hdlc_decoder_t decoder;
	uint8_t outbuf[255];
	int olen;

	for(uint16_t i=0; i<= 0xFF; i++)
	{
		uint8_t len;
		uint8_t ibuf1[4];
		uint8_t out[] = {(uint8_t)i};

		switch(i)
		{
			case 0x7D: // Escape character
			case 0x7E: // Framing character
				ibuf1[0] = 0x7E;
				ibuf1[1] = 0x7D;
				ibuf1[2] = 0x20 ^ i;
				ibuf1[3] = 0x7E;
				len = 4;
			break;
			default:
				ibuf1[0] = 0x7E;
				ibuf1[1] = i;
				ibuf1[2] = 0x7E;
				len = 3;
			break;
		}


		// Test decoder decode -------------------------------------------------
		hdlc_decoder_init(&decoder);

		memset(outbuf, 0, sizeof(outbuf));
		for (uint8_t n = 0; n < len; n++)
		{
			olen = hdlc_decoder_append(&decoder, ibuf1[n]);
			TEST_ASSERT_GREATER_THAN_INT(-1, olen);

			if (olen > 0)
			{
				TEST_ASSERT_EQUAL_UINT8(len, n + 1);
				break;
			}
		}

		TEST_ASSERT_EQUAL_INT(sizeof(out), olen);

		int clen = hdlc_decoder_copy(&decoder, outbuf, sizeof(outbuf));

		TEST_ASSERT_EQUAL_INT(olen, clen);

		TEST_ASSERT_EQUAL_UINT8_ARRAY(out, outbuf, sizeof(out));
	}
}
