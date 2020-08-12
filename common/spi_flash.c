/*
 * Basic JEDEC SPI flash wrapper.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 * @author Veiko Rütter, Raido Pahtma
 */
#include "spi_flash.h"
#include "retargetspi.h"
#include <stdio.h>

#define SPI_FLASH_CS               0
#define SPI_FLASH_PARTITIONS_COUNT 3

static struct spi_flash_partitions_struct spi_flash_partitions[SPI_FLASH_PARTITIONS_COUNT];
static int spi_flash_sleeping = 1;

#ifdef SPI_FLASH_TRACK_SUSPENDED_TIME
#include "cmsis_os2_ext.h"
static osMutexId_t m_suspend_mutex;
static uint64_t m_suspend_time;      // Time spent in the suspended state
static uint32_t m_suspend_timestamp; // When the flash was last suspended
#endif//SPI_FLASH_TRACK_SUSPENDED_TIME

void spi_flash_init(void)
{
	int i;

	spi_flash_partitions[0].start = 0x0;
	spi_flash_partitions[0].end = 0x4000;

	spi_flash_partitions[1].start = 0x4000;
	spi_flash_partitions[1].end = 0x100000;

	spi_flash_partitions[2].start = 0x100000;
	spi_flash_partitions[2].end = 0x800000;

	#ifdef SPI_FLASH_TRACK_SUSPENDED_TIME
	m_suspend_mutex = osMutexNew(NULL);
	#endif//SPI_FLASH_TRACK_SUSPENDED_TIME

	spi_flash_resume();
	for (i = 0; i < SPI_FLASH_PARTITIONS_COUNT; i++)
	{
		spi_flash_partitions[i].size = spi_flash_partitions[i].end - spi_flash_partitions[i].start;
	}
}

void spi_flash_suspend(void)
{
	// Put FLASH into deep sleep
	if (spi_flash_sleeping)
	{
		return;
	}
	spi_flash_wait_busy();
	RETARGET_SpiTransferHalf(SPI_FLASH_CS, "\xB9", 1, NULL, 0);
	spi_flash_sleeping = 1;

	#ifdef SPI_FLASH_TRACK_SUSPENDED_TIME
	while(osOK != osMutexAcquire(m_suspend_mutex, osWaitForever));
	m_suspend_timestamp = osCounterGetMilli();
	osMutexRelease(m_suspend_mutex);
	#endif//SPI_FLASH_TRACK_SUSPENDED_TIME
}

void spi_flash_resume(void)
{
	int i, j;
	uint8_t buffer[3];
	if (!spi_flash_sleeping)
	{
		return;
	}

	#ifdef SPI_FLASH_TRACK_SUSPENDED_TIME
	while(osOK != osMutexAcquire(m_suspend_mutex, osWaitForever));
	m_suspend_time += (osCounterGetMilli() - m_suspend_timestamp);
	osMutexRelease(m_suspend_mutex);
	#endif//SPI_FLASH_TRACK_SUSPENDED_TIME

	// Wake FLASH chip from deep sleep
	for (i = 0; i < 3; i++)
	{
		RETARGET_SpiTransferHalf(SPI_FLASH_CS, "\xAB", 1, NULL, 0);
		for(j = 0; j < 1000; j++)
		{
			buffer[0] = 0xFF;
			RETARGET_SpiTransferHalf(SPI_FLASH_CS, "\x9F", 1, buffer, 3);
			if((buffer[0] != 0xFF) && (buffer[0] != 0x00))
			{
				break;
			}
		}
	}
	spi_flash_sleeping = 0;
}

uint64_t spi_flash_suspended_time(void)
{
	uint64_t st = 0;

	#ifdef SPI_FLASH_TRACK_SUSPENDED_TIME
	while(osOK != osMutexAcquire(m_suspend_mutex, osWaitForever));
	if (spi_flash_sleeping)
	{
		st = m_suspend_time + (osCounterGetMilli() - m_suspend_timestamp);
	}
	else
	{
		st = m_suspend_time;
	}
	osMutexRelease(m_suspend_mutex);
	#endif//SPI_FLASH_TRACK_SUSPENDED_TIME

	return st;
}

void spi_flash_cmd(uint8_t cmd)
{
	RETARGET_SpiTransferHalf(SPI_FLASH_CS, &cmd, 1, NULL, 0);
}

uint8_t spi_flash_status(void)
{
	uint8_t c;
	RETARGET_SpiTransferHalf(SPI_FLASH_CS, "\x05", 1, &c, 1);
	return c;
}

void spi_flash_wait_busy(void)
{
	while(spi_flash_status() & 0x01);
}

void spi_flash_normalize(void)
{
}

void spi_flash_wait_wel(void)
{
	while(!(spi_flash_status() & 0x02));
}

void spi_flash_mass_erase(void)
{
	spi_flash_resume();
	spi_flash_wait_busy();
	spi_flash_cmd(0x06);
	spi_flash_wait_wel();
	spi_flash_cmd(0xC7);
	spi_flash_wait_busy();
	spi_flash_cmd(0x04);
	spi_flash_wait_busy();
}

int32_t spi_flash_read(int partition, uint32_t addr, uint32_t size, uint8_t * dst)
{
	uint8_t buffer[5];
	uint32_t len;

	if (partition >= SPI_FLASH_PARTITIONS_COUNT)
	{
		return -1;
	}

	if (addr > spi_flash_partitions[partition].size)
	{
		return -1;
	}

	spi_flash_resume();

	len = spi_flash_partitions[partition].size - addr;
	if (size > len)
	{
		size = len;
	}

	addr += spi_flash_partitions[partition].start;

	spi_flash_wait_busy();
	buffer[0] = 0x0B;
	buffer[1] = ((addr >> 16) & 0xFF);
	buffer[2] = ((addr >> 8) & 0xFF);
	buffer[3] = ((addr >> 0) & 0xFF);
	buffer[4] = 0xFF;
	RETARGET_SpiTransferHalf(SPI_FLASH_CS, buffer, 5, dst, size);
	return size;
}

int32_t spi_flash_write(int partition, uint32_t addr, uint32_t size, uint8_t * src)
{
	static uint8_t buffer[260];
	uint32_t len;
	uint32_t i;

	if (partition >= SPI_FLASH_PARTITIONS_COUNT)
	{
		return -1;
	}

	if (addr > spi_flash_partitions[partition].size)
	{
		return -1;
	}

	spi_flash_resume();

	len = spi_flash_partitions[partition].size - addr;
	if (size > len)
	{
		size = len;
	}

	addr += spi_flash_partitions[partition].start;

	spi_flash_wait_busy();
	spi_flash_cmd(0x06);
	spi_flash_wait_wel();
	buffer[0] = 0x02;
	buffer[1] = ((addr >> 16) & 0xFF);
	buffer[2] = ((addr >> 8) & 0xFF);
	buffer[3] = ((addr >> 0) & 0xFF);
	if(size > 256)
	{
		size = 256;
	}
	for(i = 0; i < size; i++)
	{
		buffer[4 + i] = src[i];
	}
	RETARGET_SpiTransferHalf(SPI_FLASH_CS, buffer, 4 + size, NULL, 0);
	spi_flash_wait_busy();
	spi_flash_cmd(0x04);
	spi_flash_wait_busy();
	return size;
}

int32_t spi_flash_erase(int partition, uint32_t addr, uint32_t size) // sector 4KB (0x20), half block 32KB (0x52), block 64KB (0xD8)
{
	uint8_t buffer[4];
	uint32_t len;

	if (partition >= SPI_FLASH_PARTITIONS_COUNT)
	{
		return -1;
	}

	if (addr > spi_flash_partitions[partition].size)
	{
		return -1;
	}

	len = spi_flash_partitions[partition].size - addr;
	if (size > len)
	{
		size = len;
	}

	if (size < 4096)
	{
		return -1;
	}

	addr += spi_flash_partitions[partition].start;

	spi_flash_resume();

	spi_flash_wait_busy();
	spi_flash_cmd(0x06);
	spi_flash_wait_wel();
	buffer[0] = 0x20;
	buffer[1] = ((addr >> 16) & 0xFF);
	buffer[2] = ((addr >> 8) & 0xFF);
	buffer[3] = ((addr >> 0) & 0xFF);
	RETARGET_SpiTransferHalf(SPI_FLASH_CS, buffer, 4, NULL, 0);
	spi_flash_wait_busy();
	spi_flash_cmd(0x04);
	spi_flash_wait_busy();
	return size;
}

int32_t spi_flash_size(int partition)
{
	if (partition >= SPI_FLASH_PARTITIONS_COUNT)
	{
		return -1;
	}
	return(spi_flash_partitions[partition].size);
}

int32_t spi_flash_erase_size(int partition)
{
	if (partition >= SPI_FLASH_PARTITIONS_COUNT)
	{
		return -1;
	}
	return(4096);
}

void spi_flash_lock()
{
	RETARGET_SpiTransactionLock(SPI_FLASH_CS);
}

void spi_flash_unlock()
{
	RETARGET_SpiTransactionUnlock(SPI_FLASH_CS);
}

