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
        debug1("ID HiWtrMrk State Prio Name");
        /* Generate raw status information about each task. */
        uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);

        for (x=0; x<uxArraySize; x++)
        {
           debug1("%"PRIu32" %3"PRIu32"/%4"PRIu32" %"PRIu16" %"PRIu32" %s",
                                        pxTaskStatusArray[x].xTaskNumber,
                                        pxTaskStatusArray[x].usStackHighWaterMark,
                                        pxTaskStatusArray[x].usStackHighWaterMark*4,
                                        pxTaskStatusArray[x].eCurrentState,
                                        pxTaskStatusArray[x].uxCurrentPriority,
                                        pxTaskStatusArray[x].pcTaskName);
        }

        vPortFree(pxTaskStatusArray);
    }
}


void basic_rtos_heap_stats (void)
{
    HeapStats_t hs;
    vPortGetHeapStats(&hs);
    debug1("hs s:%d lf:%d sf:%d nf:%d mbf:%d na:%d nf:%d",
        (int)hs.xAvailableHeapSpaceInBytes,
        (int)hs.xSizeOfLargestFreeBlockInBytes,
        (int)hs.xSizeOfSmallestFreeBlockInBytes,
        (int)hs.xNumberOfFreeBlocks,
        (int)hs.xMinimumEverFreeBytesRemaining,
        (int)hs.xNumberOfSuccessfulAllocations,
        (int)hs.xNumberOfSuccessfulFrees);
}
