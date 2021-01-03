/**
 * Debug logger for SiLabs EFR32 with fwrite.
 *
 * Minimal implementation for use before RTSO, NOT thread-safe. Blocking.
 * Switch to logger_fwrite or logger_ldma after kernel starts.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
*/
#ifndef LOGGER_FWRITE_BASIC_H_
#define LOGGER_FWRITE_BASIC_H_

int logger_fwrite_basic (const char *ptr, int len);

#endif//LOGGER_FWRITE_BASIC_H_
