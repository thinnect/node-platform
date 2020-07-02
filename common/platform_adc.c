/**
 * Platform ADC access management implementation with CMSIS semaphores.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "platform_adc.h"

#include "cmsis_os2.h"

#include "platform_io.h"

#ifndef PLATFORM_ADC_COUNT
#error PLATFORM_ADC_COUNT is not defined, add it to platform_io.h.
#endif//PLATFORM_ADC_COUNT

static osSemaphoreId_t m_adc_semaphores[PLATFORM_ADC_COUNT];
static void * m_adcs[] = PLATFORM_ADCS;

void platform_adc_management_init ()
{
	for(uint8_t i=0;i<PLATFORM_ADC_COUNT;i++)
	{
		m_adc_semaphores[i] = osSemaphoreNew(1, 1, NULL);
	}
}

uint8_t platform_adc_count ()
{
	return PLATFORM_ADC_COUNT;
}

bool platform_adc_request (void * adc, uint32_t timeout_ms)
{
	for(uint8_t i=0;i<PLATFORM_ADC_COUNT;i++)
	{
		if (adc == m_adcs[i])
		{
			if (osOK == osSemaphoreAcquire(m_adc_semaphores[i], timeout_ms))
			{
				return true;
			}
			break;
		}
	}
	return false;
}

void platform_adc_release (void * adc)
{
	for(uint8_t i=0;i<PLATFORM_ADC_COUNT;i++)
	{
		if (adc == m_adcs[i])
		{
			osSemaphoreRelease(m_adc_semaphores[i]);
			break;
		}
	}
}
