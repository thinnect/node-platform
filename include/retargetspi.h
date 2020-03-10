/*
 * SiLabs RETARGET_Serial inspired SPI retargeting solution.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Veiko Rütter
 */
#pragma once

#include <stdint.h>

/**
 * Initialize the SPI interface spicified in the retargetspi.h header.
 */
void RETARGET_SpiInit(void);

/**
 * Deinitialize the SPI interface spicified in the retargetspi.h header.
 */
void RETARGET_SpiDisable(void);

/**
 * Acquire the mutex according to cs. Blocking function.

 * @param cs A chip select.
 */
void RETARGET_SpiTransactionLock(int cs);

/**
 * Release the mutex according to cs.

 * @param cs A chip select.
 */
void RETARGET_SpiTransactionUnlock(int cs);

/**
 * Write len bytes and read len bytes at the same time.
 *
 * @param cs A chip select.
 * @param out buffer to write.
 * @param in buffer to read data into.
 * @param len number of bytes to transfer.
 * @return 0 on success or negative on error.
 */
int RETARGET_SpiTransfer(int cs, const void *out, void *in, int32_t len);

/**
 * Write out_len bytes and after that read in_len bytes.
 *
 * @param cs A chip select.
 * @param out buffer to write.
 * @param out_len number of bytes to write.
 * @param in buffer to read data into.
 * @param in_len number of bytes to read.
 * @return 0 on success or negative on error.
 */
int RETARGET_SpiTransferHalf(int cs,
                             const void *out, int32_t out_len,
                             void *in, int32_t in_len);

