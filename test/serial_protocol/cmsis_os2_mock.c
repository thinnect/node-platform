#include "cmsis_os2.h"

osMutexId_t osMutexNew(void * args)
{
	return NULL;
}

void osMutexAcquire(osMutexId_t m, int wait)
{

}
void osMutexRelease(osMutexId_t m)
{

}

osTimerId_t osTimerNew(void* cb, int tp, void * user, void * args)
{
	return NULL;
}

void osTimerStart(osTimerId_t tmr, int t)
{

}

void osTimerStop(osTimerId_t tmr)
{

}
