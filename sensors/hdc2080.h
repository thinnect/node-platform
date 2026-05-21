/******************************************************************************
 * @file    hdc2080.h
 * @brief   Public API for HDC2080 sensor driver
 *****************************************************************************/

#ifndef HDC2080_H
#define HDC2080_H

#include <stdint.h>

int32_t hdc2080_init (void);
int32_t hdc2080_read_manufacturer_id (uint16_t* p_manufacturer_id);
int32_t hdc2080_trigger_measurement (void);
int32_t hdc2080_read_temperature (float* temperature_c);
int32_t hdc2080_read_humidity (float* humidity_rh);
int32_t hdc2080_read_temp_hum (float* temperature_c, float* humidity_rh);
int32_t hdc2080_set_temperature_offset (uint8_t offset_bits);
int32_t hdc2080_get_temperature_offset (uint8_t *offset_bits);
float   hdc2080_offset_bits_to_celsius (uint8_t offset_bits);
uint8_t hdc2080_celsius_to_offset_bits (float offset_c);

#endif /* HDC2080_H */
