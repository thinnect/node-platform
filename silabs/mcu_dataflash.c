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
#include "em_core.h"

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
static platform_mutex_t m_mcu_dataflash_lock_mutex;

void mcu_dataflash_init (void)
{
	debug1("init");
	m_mcu_dataflash_mutex = platform_mutex_new("mcuf");
	m_mcu_dataflash_lock_mutex = platform_mutex_new("mcufl");
	debug1("initd");
}

void mcu_dataflash_mass_erase (void)
{
	mcu_dataflash_erase(0, MCU_DATAFLASH_START, MCU_DATAFLASH_SIZE);
}

int32_t mcu_dataflash_read (int partition, uint32_t addr, uint32_t size, uint8_t * dst)
{
	debug4("R %"PRIu32" L %"PRIu32, addr, size);

	addr += MCU_DATAFLASH_START;

	platform_mutex_acquire(m_mcu_dataflash_mutex);
	memcpy(dst, (void*)addr, size);
	platform_mutex_release(m_mcu_dataflash_mutex);
	return MCUDF_SUCCESS;
}

int32_t mcu_dataflash_write (int partition, uint32_t addr, uint32_t size, uint8_t * src)
{
	debug1("W %"PRIu32" L %"PRIu32, addr, size);
	debugb2("", src, (uint8_t)size);

	uint32_t misaligned = addr % 4;

	addr += MCU_DATAFLASH_START;

	platform_mutex_acquire(m_mcu_dataflash_mutex);

	MSC_Init();

	if (misaligned)
	{
		uint8_t data[4];
		uint8_t len = 4 - misaligned;

		if (size < len)
		{
			len = size;
		}

		addr = addr - misaligned;
		// Read existing data
		memcpy(data, (uint8_t*)addr, 4);
		// Overlay new data
		memcpy(&(data[misaligned]), src, len);
		// Write, result only valid if overwritten locations contained FF
		debug3("w %"PRIu32" l %"PRIu32, addr, 4);
		CORE_DECLARE_IRQ_STATE;
		CORE_ENTER_CRITICAL();
		if (mscReturnOk != MSC_WriteWord((uint32_t*)addr, data, 4))
		{
			CORE_EXIT_CRITICAL();
			err1("w %"PRIu32" l %"PRIu32, addr, 4);
			MSC_Deinit();
			platform_mutex_release(m_mcu_dataflash_mutex);
			return MCUDF_INVALID;
		}
		CORE_EXIT_CRITICAL();
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

		debug3("w %"PRIu32" l %"PRIu32, offset, 4);
		CORE_DECLARE_IRQ_STATE;
		CORE_ENTER_CRITICAL();
		if (mscReturnOk != MSC_WriteWord((uint32_t*)offset, data, 4))
		{
			CORE_EXIT_CRITICAL();
			err1("w %"PRIu32" l %"PRIu32, offset, 4);
			MSC_Deinit();
			platform_mutex_release(m_mcu_dataflash_mutex);
			return MCUDF_INVALID;
		}
		CORE_EXIT_CRITICAL();
		size -= leftover;
	}

	if (size > 0)
	{
		if ((0 == size % 4)&&(0 == addr % 4))
		{
			debug3("w %"PRIu32" l %"PRIu32, addr, size);
			CORE_DECLARE_IRQ_STATE;
			CORE_ENTER_CRITICAL();
			if (mscReturnOk != MSC_WriteWord((uint32_t*)addr, src, size))
			{
				CORE_EXIT_CRITICAL();
				err1("w %"PRIu32" l %"PRIu32, addr, size);
				MSC_Deinit();
				platform_mutex_release(m_mcu_dataflash_mutex);
				return MCUDF_FAIL;
			}
			CORE_EXIT_CRITICAL();
		}
		else
		{
			err1("w %"PRIu32" l %"PRIu32, addr, size);
			MSC_Deinit();
			platform_mutex_release(m_mcu_dataflash_mutex);
			return MCUDF_INVALID;
		}
	}

	MSC_Deinit();
	platform_mutex_release(m_mcu_dataflash_mutex);
	return MCUDF_SUCCESS;
}

int32_t mcu_dataflash_erase (int partition, uint32_t addr, uint32_t size)
{
	addr += MCU_DATAFLASH_START;

	platform_mutex_acquire(m_mcu_dataflash_mutex);

	if (0 == addr % FLASH_PAGE_SIZE) // Check alignment
	{
		MSC_Init();
		for (uint32_t offset=addr; offset < addr+size; offset+=FLASH_PAGE_SIZE)
		{
			info1("erase %"PRIu32, offset);
			CORE_DECLARE_IRQ_STATE;
			CORE_ENTER_CRITICAL();
			if (mscReturnOk != MSC_ErasePage((uint32_t*)offset))
			{
				CORE_EXIT_CRITICAL();
				err1("erase %"PRIu32, offset);
				MSC_Deinit();
				platform_mutex_release(m_mcu_dataflash_mutex);
				return MCUDF_FAIL;
			}
			CORE_EXIT_CRITICAL();
		}
		MSC_Deinit();
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

int32_t mcu_dataflash_size (int partition)
{
	if (0 == partition)
	{
		return MCU_DATAFLASH_SIZE;
	}
	return -1;
}

int32_t mcu_dataflash_erase_size (int partition)
{
	return FLASH_PAGE_SIZE;
}

void mcu_dataflash_lock ()
{
	debug4("lck");
	platform_mutex_acquire(m_mcu_dataflash_lock_mutex);
}

void mcu_dataflash_unlock ()
{
	debug4("ulck");
	platform_mutex_release(m_mcu_dataflash_lock_mutex);
}
