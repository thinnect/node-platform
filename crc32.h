// @license MIT

#ifndef _CRC32_H_
#define _CRC32_H_

#include <stdint.h>

#define crc32(a, b) crc32_byte(a, b)

void crc32_init(void);
uint32_t crc32_byte(uint32_t crc, uint8_t data);
uint32_t crc32_block(uint32_t crc, const void *data, uint32_t len);

#endif

