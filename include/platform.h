/*
 * Microcontroller platform initialization and general functions API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_

#include <stdint.h>

/*
 * Handle coming out of reset and setting up the chip and clocks, some GPIO.
 * @return reset cause.
 */
uint32_t PLATFORM_Init();

/*
 * Set up platform specific external radio elements.
 */
void PLATFORM_RadioInit();

/*
 * Configure platform LED GPIO.
 */
void PLATFORM_LedsInit();

/*
 * Each bit is an LED.
 * |led7|led6|led5|led4|led3|LED2|LED1|LED0|
 * Platforms usually have 2-4 leds, 3 being most common.
 *
 * @param leds LEDs bitfield.
 */
void PLATFORM_LedsSet(uint8_t leds);

/*
 * Each bit is an LED.
 * |led7|led6|led5|led4|led3|LED2|LED1|LED0|
 * Platforms usually have 2-4 leds, 3 being most common.
 *
 * @return LEDs bitfield.
 */
uint8_t PLATFORM_LedsGet();

#endif//_PLATFORM_H_
