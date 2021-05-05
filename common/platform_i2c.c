/**
 * Platform I2C access management implementation with CMSIS semaphores.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "platform_i2c.h"

#include "cmsis_os2.h"

#include "platform_io.h"

#ifndef PLATFORM_I2C_COUNT
#error PLATFORM_I2C_COUNT is not defined, add it to platform_io.h.
#endif//PLATFORM_I2C_COUNT

static osSemaphoreId_t m_i2c_semaphores[PLATFORM_I2C_COUNT];
static void * m_i2cs[] = PLATFORM_I2CS;

void platform_i2c_management_init ()
{
	for(uint8_t i=0;i<PLATFORM_I2C_COUNT;i++)
	{
		m_i2c_semaphores[i] = osSemaphoreNew(1, 1, NULL);
	}
}

uint8_t platform_i2c_count ()
{
	return PLATFORM_I2C_COUNT;
}

bool platform_i2c_request (void * dev, uint32_t timeout_ms)
{
	for(uint8_t i=0;i<PLATFORM_I2C_COUNT;i++)
	{
		if (dev == m_i2cs[i])
		{
			if (osOK == osSemaphoreAcquire(m_i2c_semaphores[i], timeout_ms))
			{
				return true;
			}
			break;
		}
	}
	return false;
}

void platform_i2c_release (void * dev)
{
	for(uint8_t i=0;i<PLATFORM_I2C_COUNT;i++)
	{
		if (dev == m_i2cs[i])
		{
			osSemaphoreRelease(m_i2c_semaphores[i]);
			break;
		}
	}
}
