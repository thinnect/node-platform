/**
 * Debug logger for SiLabs EFR32 with fwrite.
 *
 * Minimal implementation, thread-safe and blocking,
 * consider using the LDMA version instead.
 *
 * Copyright Thinnect Inc. 2019
 * @author Konstantin Bilozor, Raido Pahtma
 * @license MIT
*/
#ifndef LOGGER_FWRITE_H_
#define LOGGER_FWRITE_H_

#include "logger_fwrite_basic.h"

int logger_fwrite_init();
int logger_fwrite(const char *ptr, int len);

#endif//LOGGER_FWRITE_H_
