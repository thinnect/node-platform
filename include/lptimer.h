/**
 * Universal low-power timers.
 * lpTimer timers are implemented on top of a peripheral that continues to run
 * in lower energy modes and can wake the MCU to resume OS operations.
 *
 * lpTimers always operate in milliseconds.
 *
 * Memory for lpTimers is always allocated by the user. The lpTimer subsystem
 * is automatically initialized when the first timer is initialized and
 * automatically deinitialized when the last timer is deinitialized.
 *
 * lpTimers offer virtualization, but also priority levels which determine
 * the order in which the timer callbacks are called in situations where
 * several callbacks are pending, however already running callbacks are not
 * interrupted. The default and lowest priority level is 0.
 *
 * Copyright Thinnect Inc. 2019
 * @license MIT
 * @author Raido Pahtma
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"

// lpTimer callback function type
typedef void (*lpTimerFunc_t) (void *argument);

// lpTimer types
typedef enum {
	lpTimerOnce     = 0,
	lpTimerPeriodic = 1
} lpTimerType_t;

// lpTimer attributes structure
typedef struct {
	const char* name;
	uint8_t     priority;
} lpTimerAttr_t;

// lpTimer structure forward-declaration
typedef struct lpTimerStruct lpTimer_t;

/**
 * Initialize and register a timer.
 * @param  timer    A pointer to memory for a timer.
 * @param  func     The callback function pointer.
 * @param  type     lpTimerOnce for one-shot or lpTimerPeriodic for periodic.
 * @param  argument An argument to the timer callback function.
 * @param  attr     lpTimer attributes or NULL for default values.
 * @return          osStatus status code.
 */
osStatus_t lpTimerInit(lpTimer_t* timer, lpTimerFunc_t func, lpTimerType_t type, void *argument, const lpTimerAttr_t* attr);

/**
 * Deinitialize a timer.
 * @param timer A pointer to a timer.
 * @return      osStatus status code.
 */
osStatus_t lpTimerDeinit(lpTimer_t* timer);

/**
 * Get time, returned value silently wraps around at around ~49 days.
 * @param timer A pointer to a timer.
 * @return      Current time in milliseconds.
 */
uint32_t lpTimerGetNow(lpTimer_t* timer);

/**
 * Get the name of a timer.
 * @param timer A pointer to a timer.
 * @return      name as NULL terminated string.
 */
const char *lpTimerGetName(lpTimer_t* timer);

/**
 * Get the priority of a timer.
 * @param timer A pointer to a timer.
 * @return      The priority 0-255.
 */
uint8_t lpTimerGetPriority(lpTimer_t* timer);

/**
 * Start or restart a timer.
 * @param timer        A pointer to a timer.
 * @param milliseconds Running interval in milliseconds.
 * @return             osStatus status code.
 */
osStatus_t lpTimerStart(lpTimer_t* timer, uint32_t milliseconds);

/**
 * Stop a timer.
 * @param timer A pointer to a timer.
 * @return      osStatus status code.
 */
osStatus_t lpTimerStop(lpTimer_t* timer);

/**
 * Check if a timer is running.
 * @param timer A pointer to a timer.
 * @return      true or false.
 */
bool lpTimerIsRunning(lpTimer_t* timer);


// Include platform-specific lpTimerStruct and MCU interrupt priorities for
// architectures where relevant.
#include "platform_lptimer.h"
