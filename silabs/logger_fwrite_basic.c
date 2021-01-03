/**
 * Debug logger for SiLabs EFR32 with fwrite.
 *
 * Minimal implementation for use before RTSO, NOT thread-safe. Blocking.
 * Switch to logger_fwrite or logger_ldma after kernel starts.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
*/
#include <stdio.h>

#include "logger_fwrite_basic.h"

// Basic logger for use before kernel boot
int logger_fwrite_basic (const char *ptr, int len)
{
    fwrite(ptr, len, 1, stdout);
    fflush(stdout);
    return len;
}
