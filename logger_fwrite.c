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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"

#include "logger_fwrite.h"

static osMutexId_t log_mutex;


int logger_fwrite_init() {
	log_mutex = osMutexNew(NULL);
	return 0;
}


int logger_fwrite(const char *ptr, int len) {
	while (osMutexAcquire(log_mutex, 1000) != osOK);

	fwrite(ptr, len, 1, stdout);
	fflush(stdout);

	if (osMutexRelease(log_mutex) != osOK) {
		while (1);  // panic
	}

	return len;
}
