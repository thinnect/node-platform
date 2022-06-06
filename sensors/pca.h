/**
 * Driver for GPIO multiplexer PCA9536 from Texas Instruments
 *
 * Copyright Thinnect Inc. 2019
 * @author Konstantin Bilozor
 * @license MIT
 */
#pragma once

#define IO0 	0
#define IO1 	1
#define IO2 	2
#define IO3 	3
#define IO4     4
#define IO5     5
#define IO6     6
#define IO7     7

#define PCA_DEVICE_ADDR      0x41
#define PCA_INPUT_PORT_REG   0x00
#define PCA_OUTPUT_PORT_REG  0x01
#define PCA_POLARITY_INV_REG 0x02
#define PCA_CONFIG_REG       0x03

/**
 * Configure IO pins
 *
 * @param direction IO direction, 1 - input, 0 - output
 * @param dev_addr Device address on I2C bus.
 */
void pca9536_init(uint8_t direction, uint8_t dev_addr);

/**
 * Set output pin high
 *
 * @param pin Pin number to set high
 */
void pca9536_setOutput(uint8_t pin);

/**
 * Set output pin low
 *
 * @param pin Pin number to set low
 */
void pca9536_clearOutput(uint8_t pin);
