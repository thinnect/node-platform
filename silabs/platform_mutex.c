/**
 * Locking layer with CMSIS mutexes.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Veiko Rütter
 */

#include "platform_mutex.h"

#ifdef USE_CMSIS_OS2
#include "cmsis_os2.h"

void platform_mutex_init(char * name, platform_mutex_t mutex)
{
	osMutexAttr_t mutex_attr;
	mutex_attr.name = name;
	mutex_attr.attr_bits = osMutexRecursive|osMutexPrioInherit;
	mutex_attr.cb_mem = NULL;
	mutex_attr.cb_size = 0U;
	mutex = (void*)osMutexNew(&mutex_attr);
}

void platform_mutex_acquire(platform_mutex_t mutex)
{
	while(osOK != osMutexAcquire((osMutexId_t)mutex, osWaitForever));
}

void platform_mutex_release(platform_mutex_t mutex)
{
	osMutexRelease((osMutexId_t)mutex);
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

