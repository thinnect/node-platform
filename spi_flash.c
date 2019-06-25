#include "spi_flash.h"
#include "retargetspi.h"
#include <stdio.h>

void spi_flash_init(void){
	// Wake FLASH chip from deep sleep
	RETARGET_SpiTransferHalf(0, "\xAB", 1, NULL, 0);
	spi_flash_wait_busy();
}

void spi_flash_cmd(uint8_t cmd){
	RETARGET_SpiTransferHalf(0, &cmd, 1, NULL, 0);
}

uint8_t spi_flash_status(void){
	uint8_t c;
	RETARGET_SpiTransferHalf(0, "\x05", 1, &c, 1);
	return(c);
}

void spi_flash_wait_busy(void){
	while(spi_flash_status() & 0x01);
}

void spi_flash_normalize(void){
}

void spi_flash_wait_wel(void){
	while(!(spi_flash_status() & 0x02));
}

void spi_flash_mass_erase(void){
	spi_flash_wait_busy();
	spi_flash_cmd(0x06);
	spi_flash_wait_wel();
	spi_flash_cmd(0xC7);
	spi_flash_wait_busy();
	spi_flash_cmd(0x04);
	spi_flash_wait_busy();
}

int32_t spi_flash_read(uint32_t addr, uint32_t size, uint8_t *dst){
	uint8_t buffer[5];
	spi_flash_wait_busy();
	buffer[0] = 0x0B;
	buffer[1] = ((addr >> 16) & 0xFF);
	buffer[2] = ((addr >> 8) & 0xFF);
	buffer[3] = ((addr >> 0) & 0xFF);
	buffer[4] = 0xFF;
	RETARGET_SpiTransferHalf(0, buffer, 5, dst, size);
	return(0);
}

int32_t spi_flash_write(uint32_t addr, uint32_t size, uint8_t *src){
	static uint8_t buffer[260];
	uint32_t i;
	spi_flash_wait_busy();
	spi_flash_cmd(0x06);
	spi_flash_wait_wel();
	buffer[0] = 0x02;
	buffer[1] = ((addr >> 16) & 0xFF);
	buffer[2] = ((addr >> 8) & 0xFF);
	buffer[3] = ((addr >> 0) & 0xFF);
	if(size > 256)size = 256;
	for(i = 0; i < size; i++){
		buffer[4 + i] = src[i];
	}
	RETARGET_SpiTransferHalf(0, buffer, 4 + size, NULL, 0);
	spi_flash_wait_busy();
	spi_flash_cmd(0x04);
	spi_flash_wait_busy();
	return(0);
}

int32_t spi_flash_erase(uint32_t addr, uint32_t size){ // sector 4KB (0x20), half block 32KB (0x52), block 64KB (0xD8)
	uint8_t buffer[4];
	spi_flash_wait_busy();
	spi_flash_cmd(0x06);
	spi_flash_wait_wel();
	buffer[0] = 0x20;
	buffer[1] = ((addr >> 16) & 0xFF);
	buffer[2] = ((addr >> 8) & 0xFF);
	buffer[3] = ((addr >> 0) & 0xFF);
	RETARGET_SpiTransferHalf(0, buffer, 4, NULL, 0);
	spi_flash_wait_busy();
	spi_flash_cmd(0x04);
	spi_flash_wait_busy();
	return(0);
}

