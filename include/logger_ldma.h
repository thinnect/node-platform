/**
 * Debug logger with LDMA on EFR32 with CMSIS.
 *
 * The logger immediately starts an LDMA transfer when data arrives.
 * Data during a transfer is appended to a buffer.
 *
 * Logger is thread-safe.
 *
 * Copyright Thinnect Inc. 2019
 * @author Konstantin Bilozor, Raido Pahtma
 * @license MIT
 */
#ifndef LOGGER_LDMA_H_
#define LOGGER_LDMA_H_

#include "logger_fwrite_basic.h"

#ifndef LOGGER_LDMA_BUFFER_LENGTH
#define LOGGER_LDMA_BUFFER_LENGTH 4096
#endif//LOGGER_LDMA_BUFFER_LENGTH

#include "retargetserialconfig.h"

/*
 * Initialize the LDMA logging system.
 */
int logger_ldma_init();

/**
 * Log the specified buffer of characters.
 */
int logger_ldma(const char *ptr, int len);


#if defined(LOGGER_LDMA_LEUART0)
// Logging will be through LEUART0
#elif defined(LOGGER_LDMA_USART0)
// Logging will be through USART0
#elif defined(LOGGER_LDMA_USART1)
// Logging will be through USART1
#elif defined(LOGGER_LDMA_USART2)
// Logging will be through USART2
#else
#error "LDMA logging not configured or configuration not supported!"
#endif

#endif//LOGGER_LDMA_H_
