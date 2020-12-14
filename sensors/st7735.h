/**
 * Driver for ST7735 LCD
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#ifndef _ST7735_H_
#define _ST7735_H_

#include <stdint.h>

/**
 * Initializes ST7735 driver
 */
void st7735_init();

/**
 * Clears the LCD
 */
void st7735_clear(uint8_t on);

/**
 * Displays the cache buffer onto the LCD
 */
void st7735_refresh(uint16_t color);

/**
 * Draws a pixel
 */
void st7735_pixel(uint8_t x, uint8_t y, uint8_t on);

/**
 * Read the pixel from cache buffer
 *
 * @return Return a pixel value
 */
uint8_t st7735_point(uint8_t x, uint8_t y);

/**
 * Draws a character onto the LCD
 */
void st7735_putc(uint8_t x, uint8_t y, int c);

/**
 * Draws a string onto the LCD
 */
void st7735_puts(uint8_t x, uint8_t y, char *s);

#endif

