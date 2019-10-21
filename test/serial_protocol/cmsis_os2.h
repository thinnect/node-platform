// Mock cmsis_os2.h for testing
#ifndef CMSIS_OS_2_H_
#define CMSIS_OS_2_H_

#include <string.h>

typedef void * osMutexId_t;

typedef void * osTimerId_t;

enum RandomOsMocks {
	osTimerOnce = 1,
	osWaitForever = 0xFFFFFFFF
};

osMutexId_t osMutexNew(void * args);
void osMutexAcquire(osMutexId_t m, int wait);
void osMutexRelease(osMutexId_t m);

osTimerId_t osTimerNew(void * cb, int tp, void * user, void * args);
void osTimerStart(osTimerId_t tmr, int t);
void osTimerStop(osTimerId_t tmr);

#endif//CMSIS_OS_2_H_
