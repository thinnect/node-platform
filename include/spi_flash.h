/*
 * Basic JEDEC SPI flash wrapper API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Veiko Rütter, Raido Pahtma
 */
#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_

#include <stdint.h>
#include <stdbool.h>

struct spi_flash_partitions_struct
{
	uint32_t start;
	uint32_t end;
	uint32_t size;
};

/**
 * Initialize the SPI flash component.
 * @return true if flash working,
 *         false intialization failed - flash did not respond.
 */
bool spi_flash_init(void);

/**
 * Suspend the SPI flash to save power, resume is automatic.
 * Be sure to lock and unlock the flash around this call.
 */
void spi_flash_suspend(void);

/**
 * Explicitly wake the flash, normally done automatically.
 * @return true if flash working, false if resume failed.
 */
bool spi_flash_resume(void);

void spi_flash_cmd(uint8_t cmd);
uint8_t spi_flash_status(void);
void spi_flash_wait_busy(void);
void spi_flash_normalize(void);
void spi_flash_mass_erase(void);

int32_t spi_flash_read(int partition, uint32_t addr, uint32_t size, uint8_t * dst);
int32_t spi_flash_write(int partition, uint32_t addr, uint32_t size, uint8_t * src);
int32_t spi_flash_erase(int partition, uint32_t addr, uint32_t size);
int32_t spi_flash_size(int partition);
int32_t spi_flash_erase_size(int partition);

/**
 * Milliseconds spent in suspended state.
 * SPI_FLASH_TRACK_SUSPENDED_TIME must be defined, otherwise always returns 0.
 */
uint64_t spi_flash_suspended_time();

/**
 * Lock the SPI flash before access.
 */
void spi_flash_lock();

/**
 * Unlock the SPI flash after access.
 */
void spi_flash_unlock();

#endif//_SPI_FLASH_H_
