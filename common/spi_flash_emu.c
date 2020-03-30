/*
 * Simulate SPI flash on top of MCU internal flash.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#include "spi_flash.h"
#include "mcu_dataflash.h"

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "loglevels.h"
#define __MODUUL__ "sfemu"
#define __LOG_LEVEL__ (LOG_LEVEL_spi_flash_emu & BASE_LOG_LEVEL)
#include "log.h"

void spi_flash_init(void)
{
	debug1("init");
	mcu_dataflash_init();
}

void spi_flash_cmd(uint8_t cmd)
{
}

uint8_t spi_flash_status(void)
{
	return 0;
}

void spi_flash_wait_busy(void)
{
}

void spi_flash_normalize(void)
{
}

void spi_flash_wait_wel(void)
{
}

void spi_flash_mass_erase(void)
{
	mcu_dataflash_mass_erase();
}

int32_t spi_flash_read(uint32_t addr, uint32_t size, uint8_t * dst)
{
	return mcu_dataflash_read(addr, size, dst);
}

int32_t spi_flash_write(uint32_t addr, uint32_t size, uint8_t * src)
{
	return mcu_dataflash_write(addr, size, src);
}

int32_t spi_flash_erase(uint32_t addr, uint32_t size)
{
	return mcu_dataflash_erase(addr, size);
}

void spi_flash_lock()
{
}

void spi_flash_unlock()
{
}
