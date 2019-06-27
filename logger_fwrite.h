/**
 * Debug logger for SiLabs EFR32 with fwrite.
 *
 * Minimal implementation, thread-safe and blocking,
 * consider using the LDMA version instead.
 *
 * Copyright Thinnect Inc.
 * @author Konstantin Bilozor, Raido Pahtma
 * @license MIT
*/
#pragma once

int logger_fwrite_init();
int logger_fwrite(const char *ptr, int len);
