#ifndef _SPI_FLASH_H_
#define _SPI_FLASH_H_

#include <stdint.h>

void spi_flash_init(void);

void spi_flash_cmd(uint8_t cmd);
uint8_t spi_flash_status(void);
void spi_flash_wait_busy(void);
void spi_flash_normalize(void);
void spi_flash_mass_erase(void);

int32_t spi_flash_read(uint32_t addr, uint32_t size, uint8_t *dst);
int32_t spi_flash_write(uint32_t addr, uint32_t size, uint8_t *src);
int32_t spi_flash_erase(uint32_t addr, uint32_t size);

#endif

