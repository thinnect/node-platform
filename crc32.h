/**
 * Byte/block CRC-32 algorithm API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 */
#ifndef CRC32_H_
#define CRC32_H_

#include <stdint.h>

void crc32_init(void);
uint32_t crc32_byte(uint32_t crc, uint8_t data);
uint32_t crc32_block(uint32_t crc, const void *data, uint32_t len);

#endif//CRC32_H_
