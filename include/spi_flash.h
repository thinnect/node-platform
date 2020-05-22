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

void spi_flash_init(void);

void spi_flash_suspend(void);
void spi_flash_resume(void);

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

void spi_flash_lock();
void spi_flash_unlock();

#endif

