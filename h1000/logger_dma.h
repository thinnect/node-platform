/**
 * DMA Logging for H1002.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
*/
#ifndef LOGGER_DMA_H
#define LOGGER_DMA_H
#include <stdbool.h>

/*
 * Initialize the DMA logging system.
 */
bool logger_dma_init(void);

/**
 * Log the specified buffer of characters.
 */
int logger_dma(const char *ptr, int len);

#endif

