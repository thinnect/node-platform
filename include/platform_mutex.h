/**
 * Locking layer with CMSIS mutexes.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma, Veiko Rütter
 */

#ifndef _PLATFORM_MUTEX_H_
#define _PLATFORM_MUTEX_H_

struct platform_mutex_struct
{
	void * mutex;
	void * attr;
};

typedef struct platform_mutex_struct * platform_mutex_t;

/**
 * Initialize the mutex.

 * @param name A name of mutex.
 *
 * @return mutex or NULL on error
 */
platform_mutex_t platform_mutex_init(char * name);

/**
 * Acquire the mutex. Blocking function.

 * @param mutex A mutex.
 */
void platform_mutex_acquire(platform_mutex_t mutex);

/**
 * Release the mutex.

 * @param mutex A mutex.
 */
void platform_mutex_release(platform_mutex_t mutex);

#endif

