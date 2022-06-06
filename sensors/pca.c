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
#include "cmsis_os2.h"
#include "pca.h"

#include "loglevels.h"
#define __MODUUL__ "pca"
#define __LOG_LEVEL__ (LOG_LEVEL_pca & BASE_LOG_LEVEL)
#include "log.h"

static uint8_t pca_addr = 0;

void pca9536_init(uint8_t direction, uint8_t dev_addr)
{
	uint8_t regv;

	pca_addr = dev_addr;

	regv = direction;
	RETARGET_I2CWrite(pca_addr, PCA_CONFIG_REG, &regv, 1);
	osDelay(10);

	regv = 0; // Set all output values to 0
	RETARGET_I2CWrite(pca_addr, PCA_OUTPUT_PORT_REG, &regv, 1);
	osDelay(10);
}

void pca9536_setOutput(uint8_t pin)
{
	uint8_t regv;

	// Read current status
	RETARGET_I2CRead(pca_addr, PCA_OUTPUT_PORT_REG, &regv, 1);
	osDelay(10);

	regv = regv | (0x01 << pin);
	RETARGET_I2CWrite(pca_addr, PCA_OUTPUT_PORT_REG, &regv, 1);
	osDelay(10);
}

void pca9536_clearOutput(uint8_t pin)
{
	uint8_t regv;

	// Read current status
	RETARGET_I2CRead(pca_addr, PCA_OUTPUT_PORT_REG, &regv, 1);
	osDelay(10);

	regv = regv & ~(0x01 << pin);
	RETARGET_I2CWrite(pca_addr, PCA_OUTPUT_PORT_REG, &regv, 1);
	osDelay(10);
}
