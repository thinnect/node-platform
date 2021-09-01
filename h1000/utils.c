/**
 * Utility functions for h1002.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "utils.h"


void printpacket(uint8_t* data, uint16_t len)
{
	LOG("\r\nPrinting packet\r\n");
	for(int i = 0; i < len; i++)
	{
		LOG(" %x ",*(data + i));
	}
	LOG("\r\n");
}
