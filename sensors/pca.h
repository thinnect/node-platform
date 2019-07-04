/**
 * Driver for GPIO multiplexer PCA9536 from Texas Instruments 
 *
 * @copyright Thinnect
 * @author Konstantin Bilozor
 * @license MIT
 */
#pragma once

#define IO0 	0
#define IO1 	1
#define IO2 	2
#define IO3 	3

/**
 * Configure IO pins
 *
 * @param direction IO direction, 1 - input, 0 - output
 */
void pca9536_init(uint8_t direction);

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
