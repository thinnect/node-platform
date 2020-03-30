/*
 * SPIFFS compatible MCU flash data area access functions.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */
#ifndef MCU_DATAFLASH_H_
#define MCU_DATAFLASH_H_

#include <stdint.h>

/**
 * Initialize mcu dataflash module.
 */
void mcu_dataflash_init(void);

/**
 * Perform erase for the whole dataflash area of the MCU flash.
 */
void mcu_dataflash_mass_erase(void);

/**
 * Read from dataflash.
 * @param addr Where to read from.
 * @param size How much to read.
 * @param dst Pointer to destination buffer.
 * @return 0 for success;
 */
int32_t mcu_dataflash_read(uint32_t addr, uint32_t size, uint8_t * dst);

/**
 * Write to dataflash.
 * @param addr Where to write.
 * @param size How much to write.
 * @param src Pointer to data.
 * @return 0 for success;
 */
int32_t mcu_dataflash_write(uint32_t addr, uint32_t size, uint8_t * src);

/**
 * Erase a page from dataflash.
 * @param addr Page aligned address to erase.
 * @param size Bytes to erase, must be a multiple of page size.
 * @return 0 for success;
 */
int32_t mcu_dataflash_erase(uint32_t addr, uint32_t size);

// Error codes
enum MCUDataFlashErrors
{
	MCUDF_SUCCESS = 0,
	MCUDF_FAIL = -1,
	MCUDF_INVALID = -2
};

#endif//MCU_DATAFLASH_H_
