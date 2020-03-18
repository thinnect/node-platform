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

platform_mutex_t platform_mutex_new(char * name)
{
	platform_mutex_t mutex;
	osMutexAttr_t attr;
	attr.name = name;
	attr.attr_bits = osMutexRecursive|osMutexPrioInherit;
	attr.cb_mem = NULL;
	attr.cb_size = 0U;
	mutex = (platform_mutex_t)osMutexNew(&attr);
	return mutex;
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

