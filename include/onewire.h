/*
 * Onewire BUS API.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#pragma once

#include <stdint.h>

void onewire_init();
void onewire_deinit();

uint8_t onewire_reset();
uint8_t onewire_read();
void onewire_write(uint8_t bit);
