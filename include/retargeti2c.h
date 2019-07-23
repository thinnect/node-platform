/*
 * SiLabs RETARGET_Serial inspired I2C retargeting solution.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#pragma once

#include <stdint.h>

void RETARGET_I2CInit();
void RETARGET_I2CDeinit();
int8_t RETARGET_I2CWrite(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t count);
int8_t RETARGET_I2CRead(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t count);
