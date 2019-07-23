/**
 * Driver for ADC ADS1015 from Texas Instruments
 *
 * Copyright Thinnect Inc. 2019
 * @author Konstantin Bilozor
 * @license MIT
 */

#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>

#include "retargeti2c.h"
#include "mtimer.h"
#include "ads.h"

#include "loglevels.h"
#define __MODUUL__ "ads"
#define __LOG_LEVEL__ (LOG_LEVEL_ads & BASE_LOG_LEVEL)
#include "log.h"


uint16_t ads1015_read(uint8_t input) {
	uint8_t regv[2];
	uint16_t val;

	regv[0] = 0x40 | (input << 4); // FSR=6,144V, input between AINx and GND
	regv[1] = 0x83; // Default configuration
	RETARGET_I2CWrite(ADS_DEVICE_ADDR, ADS_CONFIG_REG, regv, 2);
	mtimer_sleep(10);

	RETARGET_I2CRead(ADS_DEVICE_ADDR, ADS_CONVERSION_REG, regv, 2);

	val = ((regv[0] << 8) + regv[1]) >> 4;
	val = (val * ADS_DEFAULT_RANGE_MV) >> ADS_BITS_NUM;
	debug1("adc voltage: %"PRIu16, val);

	mtimer_sleep(10);

	return val;
}
