#ifndef PLATFORM_H
#define PLATFORM_H

#include "gpio.h"
#include "error.h"
#include "log.h"

#define PLATFORM_BUTTON P2
#define LED_1 P31
#define LED_2 P32
#define LED_3 P33

#define LED_1_MASK 0x1
#define LED_2_MASK 0x2
#define LED_3_MASK 0x4

#define FAKE_VCC P0

int platform_init(void);
void platform_setleds(uint8_t leds);
bool platform_button(void);

#endif
