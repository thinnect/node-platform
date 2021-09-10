/**
 * Platform specific functions.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
*/
#ifndef PLATFORM_H
#define PLATFORM_H

#include "gpio.h"
#include "error.h"
#include "log.h"

#define PLATFORM_BUTTON P2
#define LED_1 P25
#define LED_2 P24
#define LED_3 P23

#define LED_1_MASK 0x1
#define LED_2_MASK 0x2
#define LED_3_MASK 0x4

#define FAKE_VCC P0

int PLATFORM_Init(void);
void PLATFORM_LedsSet(uint8_t leds);
bool PLATFORM_ButtonGet(void);

#endif

