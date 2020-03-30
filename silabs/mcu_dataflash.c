/*
 * SPIFFS compatible MCU flash data area access functions for EFR32.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "mcu_dataflash.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "platform_mutex.h"
#include "em_msc.h"

#include "loglevels.h"
#define __MODUUL__ "mcuf"
#define __LOG_LEVEL__ (LOG_LEVEL_mcu_dataflash & BASE_LOG_LEVEL)
#include "log.h"

#ifndef MCU_DATAFLASH_START
#define MCU_DATAFLASH_START (512*1024UL)
#endif//MCU_DATAFLASH_START
#ifndef MCU_DATAFLASH_SIZE
#define MCU_DATAFLASH_SIZE (448*1024UL)
#endif//MCU_DATAFLASH_SIZE

static platform_mutex_t m_mcu_dataflash_mutex;

void mcu_dataflash_init(void)
{
	debug1("init");
	m_mcu_dataflash_mutex = platform_mutex_new("mcuf");
	MSC_Init();
}

void mcu_dataflash_mass_erase(void)
{
	mcu_dataflash_erase(MCU_DATAFLASH_START, MCU_DATAFLASH_SIZE);
}

int32_t mcu_dataflash_read(uint32_t addr, uint32_t size, uint8_t * dst)
{
	debug1("R %"PRIu32" L %"PRIu32, addr, size);
	platform_mutex_acquire(m_mcu_dataflash_mutex);
	memcpy(dst, (void*)addr, size);
	platform_mutex_release(m_mcu_dataflash_mutex);
	return MCUDF_SUCCESS;
}

int32_t mcu_dataflash_write(uint32_t addr, uint32_t size, uint8_t * src)
{
	debug1("W %"PRIu32" L %"PRIu32, addr, size);
	debugb1("", src, (uint8_t)size);

	uint32_t misaligned = addr % 4;

	platform_mutex_acquire(m_mcu_dataflash_mutex);

	if (misaligned)
	{
		uint8_t data[4];
		uint8_t len = 4 - misaligned;
		addr = addr - misaligned;
		// Read existing data
		memcpy(data, (uint8_t*)addr, 4);
		// Overlay new data
		memcpy(&(data[misaligned]), src, len);
		// Write, result only valid if overwritten locations contained FF
		debug1("w %"PRIu32" l %"PRIu32, addr, 4);
		if (mscReturnOk != MSC_WriteWord((uint32_t*)addr, data, 4))
		{
			err1("w %"PRIu32" l %"PRIu32, addr, 4);
			platform_mutex_release(m_mcu_dataflash_mutex);
			return MCUDF_INVALID;
		}
		addr += 4;
		src += len;
		size -= len;
	}

	uint32_t leftover = size % 4;
	if (leftover)
	{
		uint8_t data[4];
		uint32_t offset = addr+size-leftover;
		memcpy(data, (uint32_t*)offset, 4);
		memcpy(data, (uint32_t*)(src+size-leftover), leftover);

		debug1("w %"PRIu32" l %"PRIu32, offset, 4);
		if (mscReturnOk != MSC_WriteWord((uint32_t*)offset, data, 4))
		{
			err1("w %"PRIu32" l %"PRIu32, offset, 4);
			platform_mutex_release(m_mcu_dataflash_mutex);
			return MCUDF_INVALID;
		}
		size -= leftover;
	}

	if (size > 0)
	{
		if ((0 == size % 4)&&(0 == addr % 4))
		{
			debug1("w %"PRIu32" l %"PRIu32, addr, size);
			if (mscReturnOk != MSC_WriteWord((uint32_t*)addr, src, size))
			{
				err1("w %"PRIu32" l %"PRIu32, addr, size);
				platform_mutex_release(m_mcu_dataflash_mutex);
				return MCUDF_FAIL;
			}
		}
		else
		{
			err1("w %"PRIu32" l %"PRIu32, addr, size);
			platform_mutex_release(m_mcu_dataflash_mutex);
			return MCUDF_INVALID;
		}
	}

	platform_mutex_release(m_mcu_dataflash_mutex);
	return MCUDF_SUCCESS;
}

int32_t mcu_dataflash_erase(uint32_t addr, uint32_t size)
{
	platform_mutex_acquire(m_mcu_dataflash_mutex);

	if (0 == addr % FLASH_PAGE_SIZE) // Check alignment
	{
		for (uint32_t offset=addr;offset < size;offset+=FLASH_PAGE_SIZE)
		{
			debug1("erase %"PRIu32, offset);
			if (mscReturnOk != MSC_ErasePage((uint32_t*)offset))
			{
				err1("erase %"PRIu32, offset);
				platform_mutex_release(m_mcu_dataflash_mutex);
				return MCUDF_FAIL;
			}
		}
	}
	else
	{
		err1("erase %"PRIu32, addr);
		platform_mutex_release(m_mcu_dataflash_mutex);
		return MCUDF_INVALID;
	}

	platform_mutex_release(m_mcu_dataflash_mutex);
	return MCUDF_SUCCESS;
}
