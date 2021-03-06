/**
 * Driver for GPIO multiplexer PCA9536 from Texas Instruments
 *
 * Copyright Thinnect Inc. 2019
 * @author Konstantin Bilozor
 * @license MIT
 */

#include <stdint.h>
#include <stdbool.h>

#include "retargeti2c.h"
#include "mtimer.h"
#include "pca.h"

#include "loglevels.h"
#define __MODUUL__ "pca"
#define __LOG_LEVEL__ (LOG_LEVEL_pca & BASE_LOG_LEVEL)
#include "log.h"


void pca9536_init(uint8_t direction) {
	uint8_t regv;

	regv = direction;
	RETARGET_I2CWrite(PCA_DEVICE_ADDR, PCA_CONFIG_REG, &regv, 1);
	mtimer_sleep(10);

	regv = 0; // Set all output values to 0
	RETARGET_I2CWrite(PCA_DEVICE_ADDR, PCA_OUTPUT_PORT_REG, &regv, 1);
	mtimer_sleep(10);
}

void pca9536_setOutput(uint8_t pin) {
	uint8_t regv;

	// Read current status
	RETARGET_I2CRead(PCA_DEVICE_ADDR, PCA_OUTPUT_PORT_REG, &regv, 1);
	mtimer_sleep(10);

	regv = regv | (0x01 << pin);
	RETARGET_I2CWrite(PCA_DEVICE_ADDR, PCA_OUTPUT_PORT_REG, &regv, 1);
	mtimer_sleep(10);
}

void pca9536_clearOutput(uint8_t pin) {
	uint8_t regv;

	// Read current status
	RETARGET_I2CRead(PCA_DEVICE_ADDR, PCA_OUTPUT_PORT_REG, &regv, 1);
	mtimer_sleep(10);

	regv = regv & ~(0x01 << pin);
	RETARGET_I2CWrite(PCA_DEVICE_ADDR, PCA_OUTPUT_PORT_REG, &regv, 1);
	mtimer_sleep(10);
}
