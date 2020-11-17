/**
 * Gather and print RTOS threads statistics.
 *
 * Copyright Thinnect Inc. 2020
 * @license MIT
 */

#include "basic_rtos_threads_stats.h"

#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "loglevels.h"
#define __MODUUL__ "thrds"
#define __LOG_LEVEL__ (LOG_LEVEL_basic_rtos_threads_stats & BASE_LOG_LEVEL)
#include "log.h"


void basic_rtos_threads_stats (void)
{
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;

    uxArraySize = uxTaskGetNumberOfTasks();
    debug1("Number of tasks: %"PRIu32, (uint32_t)uxArraySize);
    debug1("Free heap size in bytes: %"PRIu32, (uint32_t)xPortGetFreeHeapSize());

    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    if (pxTaskStatusArray != NULL)
    {
        debug1("ID\t WtrMrk\t State\t Prio\t Name");
        /* Generate raw status information about each task. */
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);

        for (x=0; x<uxArraySize; x++)
        {
           debug1("%"PRIu32"\t %"PRIu32"\t %"PRIu16"\t %"PRIu32"\t %s\t",
                                        pxTaskStatusArray[x].xTaskNumber,
                                        pxTaskStatusArray[x].usStackHighWaterMark,
                                        pxTaskStatusArray[x].eCurrentState,
                                        pxTaskStatusArray[x].uxCurrentPriority,
                                        pxTaskStatusArray[x].pcTaskName);
        }

        vPortFree(pxTaskStatusArray);
    }
}
