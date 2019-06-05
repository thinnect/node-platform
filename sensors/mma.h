/*
 * Really basic driver for the NXP MMA8653FC.
 * https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
 *
 * Currently only supports polling the Landscape/Portrait register.
 *
 * Copyright Thinnect Inc.
 * @author Raido Pahtma
 * @license MIT
 */
#pragma once

/**
 * Read the WHO_AM_I device ID register, should return 0x5A.
 */
uint8_t mma_read_id();

/**
 * Activate Landscape/Portrait monitoring and enter ACTIVE mode.
 */
uint8_t mma_activate();

/**
 * Read the Landscape/Portrait register, return the orientation bits.
 */
uint8_t mma_read_orientation();
