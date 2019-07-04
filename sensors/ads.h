/**
 * Driver for ADC ADS1015 from Texas Instruments 
 *
 * @copyright Thinnect
 * @author Konstantin Bilozor
 * @license MIT
 */
#pragma once

#define AIN0   0
#define AIN1   1
#define AIN2   2
#define AIN3   3

#define ADS_DEFAULT_RANGE_MV  6144
#define ADS_BITS_NUM 11

/**
 * Read data from ADS1015
 *
 * @param input Select input to read data from
 *
 * @return Returns voltage in mV
 */
uint16_t ads1015_read(uint8_t input);
