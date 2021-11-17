/**
 * Platform specific functions.
 *
 * Copyright Thinnect Inc. 2021
 * @license MIT
*/ 
#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>

void PLATFORM_Init (void);
void PLATFORM_LedsSet (uint8_t leds);
bool PLATFORM_ButtonGet (void);

#endif//PLATFORM_H

