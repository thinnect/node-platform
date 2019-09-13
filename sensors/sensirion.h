#ifndef _SENSIRION_H_
#define _SENSIRION_H_

#include <stdint.h>

int16_t sensirion_i2c_select_bus(uint8_t bus_idx);
void sensirion_i2c_init(void);
void sensirion_i2c_release(void);
int8_t sensirion_i2c_read(uint8_t address, uint8_t *data, uint16_t count);
int8_t sensirion_i2c_write(uint8_t address, const uint8_t *data, uint16_t count);
void sensirion_sleep_usec(uint32_t useconds);

#endif

