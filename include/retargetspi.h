/*
 * SiLabs RETARGET_Serial inspired SPI retargeting solution.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#pragma once

#include <stdint.h>

void RETARGET_SpiInit(void);

void RETARGET_SpiDisable(void);

int RETARGET_SpiTransfer(int cs, const void *out, void *in, int32_t len);

int RETARGET_SpiTransferHalf(int cs,
                             const void *out, int32_t out_len,
                             void *in, int32_t in_len);