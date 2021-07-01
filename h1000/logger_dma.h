
#include <stdbool.h>

/*
 * Initialize the DMA logging system.
 */
bool logger_dma_init();

/**
 * Log the specified buffer of characters.
 */
int logger_dma(const char *ptr, int len);

