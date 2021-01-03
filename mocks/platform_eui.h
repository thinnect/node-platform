/**
 * Mock to retreive fake platform eui.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef PLATFORM_EUI_H_
#define PLATFORM_EUI_H_

#include <stdbool.h>
#include <string.h>

#include "eui64.h"


static inline bool platform_eui(uint8_t eui[IEEE_EUI64_LENGTH])
{
	eui[0] = 0x12;
	eui[1] = 0x34;
	eui[2] = 0x56;
	eui[3] = 0x78;
	eui[4] = 0xDE;
	eui[5] = 0xAD;
	eui[6] = 0xBE;
	eui[7] = 0xEF;

	return true;
}

#endif//PLATFORM_EUI_H_
