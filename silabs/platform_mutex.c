/**
 * Locking layer with CMSIS mutexes.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Veiko Rütter
 */

#include "platform_mutex.h"
#include <stdlib.h>

#ifdef USE_CMSIS_OS2
#include "cmsis_os2.h"

platform_mutex_t platform_mutex_init(char * name)
{
	platform_mutex_t mutex;
	mutex = malloc(sizeof(struct platform_mutex_struct));
	if(mutex == NULL)
	{
		return NULL;
	}
	mutex->attr = malloc(sizeof(osMutexAttr_t));
	if(mutex->attr == NULL)
	{
		free(mutex);
		return NULL;
	}
	((osMutexAttr_t *)(mutex->attr))->name = name;
	((osMutexAttr_t *)(mutex->attr))->attr_bits = osMutexRecursive|osMutexPrioInherit;
	((osMutexAttr_t *)(mutex->attr))->cb_mem = NULL;
	((osMutexAttr_t *)(mutex->attr))->cb_size = 0U;
	mutex->mutex = (void*)osMutexNew((osMutexAttr_t *)mutex->attr);
	if(mutex->mutex == NULL){
		free(mutex->attr);
		free(mutex);
		return NULL;
	}
	return mutex;
}

void platform_mutex_acquire(platform_mutex_t mutex)
{
	while(osOK != osMutexAcquire((osMutexId_t)mutex->mutex, osWaitForever));
}

void platform_mutex_release(platform_mutex_t mutex)
{
	osMutexRelease((osMutexId_t)mutex->mutex);
}

#else // USE_CMSIS_OS2

void platform_mutex_init(char * name, platform_mutex_t mutex)
{
}

void platform_mutex_acquire(platform_mutex_t mutex)
{
}

void platform_mutex_release(platform_mutex_t mutex)
{
}

#endif // USE_CMSIS_OS2

