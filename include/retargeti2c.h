/*
 * SiLabs RETARGET_Serial inspired I2C retargeting solution.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */

#ifndef _RETARGET_I2C_H_
#define _RETARGET_I2C_H_

#include <stdint.h>

/**
 * Initialize the I2C interface spicified in the retargeti2c.h header.
 */
void RETARGET_I2CInit();

/**
 * Deinitialize the I2C interface spicified in the retargeti2c.h header.
 */
void RETARGET_I2CDeinit();

/**
 * Acquire the mutex for a transaction. Blocking function.
 */
void RETARGET_I2CTransactionLock();

/**
 * Release the mutex.
 */
void RETARGET_I2CTransactionUnlock();

/**
 * Write 1 byte (regAddr) to set register and read 'count' bytes of data in a
 * single transaction.
 *
 * @param devAddr 7-bit I2C device address.
 * @param regAddr byte value written before the read, usually to set register.
 * @param regData buffer to read data into, must be at least 'count' bytes.
 * @param count number of bytes to read.
 * @return 0 for success.
 */
int8_t RETARGET_I2CRead(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t count);

/**
 * Write 1 byte (regAddr) to set register and write 'count' bytes of data in a
 * single transaction.
 *
 * @param devAddr 7-bit I2C device address.
 * @param regAddr byte value written before the write, usually to set register.
 * @param regData buffer to write to device.
 * @param count number of bytes to write.
 * @return 0 for success.
 */
int8_t RETARGET_I2CWrite(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t count);

/**
 * Write wcount bytes (to set register) and read 'rcount' bytes of data in a
 * single transaction.
 * Either wcount or rcount is allowed to be 0.
 *
 * @param devAddr 7-bit I2C device address.
 * @param wData buffer to write to device (multi-byte register addresses).
 * @param wCount number of bytes to write.
 * @param rData buffer to read data into, must be at least 'rCount' bytes.
 * @param rCount number of bytes to read.
 * @return 0 for success.
 */
int8_t RETARGET_I2CWriteRead(uint8_t devAddr, uint8_t* wData, uint16_t wCount, uint8_t *rData, uint16_t rCount);

#endif

