/**
 * IEEE EUI-64 functions.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "eui64.h"

#include <string.h>

bool eui64_is_zeros(const ieee_eui64_t * eui)
{
	uint8_t i;
	for (i=0;i<IEEE_EUI64_LENGTH;i++)
	{
		if (0 != eui->data[i])
		{
			return false;
		}
	}
	return true;
}

void eui64_set_zeros(ieee_eui64_t * eui)
{
	memset(eui->data, 0, IEEE_EUI64_LENGTH);
}

bool eui64_is_ones(const ieee_eui64_t * eui)
{
	uint8_t i;
	for (i=0;i<IEEE_EUI64_LENGTH;i++)
	{
		if (0xFF != eui->data[i])
		{
			return false;
		}
	}
	return true;
}

void eui64_set_ones(ieee_eui64_t * eui)
{
	memset(eui->data, 0xFF, IEEE_EUI64_LENGTH);
}

void eui64_set(ieee_eui64_t * eui, const uint8_t data[IEEE_EUI64_LENGTH])
{
	memcpy(eui->data, data, IEEE_EUI64_LENGTH);
}

void eui64_get(const ieee_eui64_t * eui, uint8_t data[IEEE_EUI64_LENGTH])
{
	memcpy(data, eui->data, IEEE_EUI64_LENGTH);
}

int eui64_compare(const ieee_eui64_t * eui1, const ieee_eui64_t * eui2)
{
	return memcmp(eui1->data, eui2->data, IEEE_EUI64_LENGTH);
}
