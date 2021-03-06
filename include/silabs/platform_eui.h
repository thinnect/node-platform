/**
 * Retrives and combines EUI64 from device registers.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef PLATFORM_EUI_H_
#define PLATFORM_EUI_H_

#include <stdbool.h>
#include <string.h>

#include "eui64.h"
#include "device.h"


static inline bool platform_eui(uint8_t eui[IEEE_EUI64_LENGTH])
{
	uint8_t zeros[IEEE_EUI64_LENGTH] = {0};
	uint8_t buff[IEEE_EUI64_LENGTH] = {[0 ... IEEE_EUI64_LENGTH-1] = 0xFF};

#ifdef EFR32_SERIES1
	eui[0] = DEVINFO->EUI48H >> 8;
	eui[1] = DEVINFO->EUI48H >> 0;
	eui[2] = DEVINFO->EUI48L >> 24;
	eui[3] = 0xFF;
	eui[4] = 0xFE;
	eui[5] = DEVINFO->EUI48L >> 16;
	eui[6] = DEVINFO->EUI48L >> 8;
	eui[7] = DEVINFO->EUI48L >> 0;
#elif defined EFR32_SERIES2
	eui[0] = DEVINFO->EUI64H >> 24;
	eui[1] = DEVINFO->EUI64H >> 16;
	eui[2] = DEVINFO->EUI64H >> 8;
	eui[3] = DEVINFO->EUI64H >> 0;
	eui[4] = DEVINFO->EUI64L >> 24;
	eui[5] = DEVINFO->EUI64L >> 16;
	eui[6] = DEVINFO->EUI64L >> 8;
	eui[7] = DEVINFO->EUI64L >> 0;
#else
	#error "Define EFR32_SERIES1 or EFR32_SERIES2"
#endif

	if (!memcmp(eui, zeros, IEEE_EUI64_LENGTH) || !memcmp(eui, buff, IEEE_EUI64_LENGTH))
	{
		return false;
	}
	return true;
}

#endif//PLATFORM_EUI_H_
