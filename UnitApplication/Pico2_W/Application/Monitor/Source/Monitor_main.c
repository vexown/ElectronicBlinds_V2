/**
 * File: Monitor_main.c
 * Description: Implementation of the main Monitor function 
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <limits.h>

/* Kernel includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* pico-sdk includes */
#include "pico/stdlib.h"
#include "hardware/timer.h"

/* WiFi includes */
#include "WiFi_TCP.h"

/* OS includes */
#include "OS_manager.h"

/* Misc includes */
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/* Structure to hold system statistics */
typedef struct 
{
    HeapStats_t heap_stats;
    uint32_t totalRunTime;
    UBaseType_t currentNumOfTasks;
} SystemStats_t;

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

static TaskStatus_t taskStatusArray[MAX_NUM_OF_TASKS];
static absolute_time_t stats_start_time;

/*******************************************************************************/
/*                          STATIC FUNCTION DECLARATIONS                       */
/*******************************************************************************/

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: Monitor_MainFunction
 * 
 * Description: Main implementation of Monitor functionalities of the system
 * It is periodically (every MONITOR_TASK_PERIOD_TICKS) executed from the Monitor 
 * FreeRTOS task
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 *
 */
void Monitor_MainFunction(void)
{
    static SystemStats_t stats;
    static configRUN_TIME_COUNTER_TYPE totalRunTime;
    static UBaseType_t arraySize, populatedArraySize;
    
    /* Get heap statistics */ 
    vPortGetHeapStats(&stats.heap_stats);
    
    /* Get task statistics */ 
    arraySize = uxTaskGetNumberOfTasks();
    stats.currentNumOfTasks = arraySize;
    totalRunTime = 0;
    populatedArraySize = uxTaskGetSystemState(taskStatusArray, MAX_NUM_OF_TASKS, &totalRunTime); // returns 0 if if the MAX_NUM_OF_TASKS is too small 
    
    LOG("=== System Statistics ===\n");
    LOG("Available Heap Space (sum of free blocks): %u bytes\n",      stats.heap_stats.xAvailableHeapSpaceInBytes);
    LOG("Size of Largest Free Block: %u bytes\n",                     stats.heap_stats.xSizeOfLargestFreeBlockInBytes);
    LOG("Size of Smallest Free Block: %u bytes\n",                    stats.heap_stats.xSizeOfSmallestFreeBlockInBytes);
    LOG("Number of Free Blocks: %u \n",                               stats.heap_stats.xNumberOfFreeBlocks);
    LOG("Minimum amount of total free memory since boot: %u bytes\n", stats.heap_stats.xMinimumEverFreeBytesRemaining);
    LOG("Number of successfull pvPortMalloc calls %u \n",             stats.heap_stats.xNumberOfSuccessfulAllocations);
    LOG("Number of successfull vPortFree calls %u \n",                stats.heap_stats.xNumberOfSuccessfulFrees);

    LOG("=== Task Statistics ===\n");
    LOG("Number of Tasks: %lu\n", stats.currentNumOfTasks);
    LOG("Name\t\tState\tPrio\tRemainingStack\tTaskNum\n");

    for (UBaseType_t task_num = 0; task_num < populatedArraySize; task_num++) 
    {
        char taskState = 'X';
        /* Cast the numeric taskState to a char representation */
        switch (taskStatusArray[task_num].eCurrentState) 
        {
            case eRunning:   taskState = 'R'; break;
            case eReady:     taskState = 'r'; break;
            case eBlocked:   taskState = 'B'; break;
            case eSuspended: taskState = 'S'; break;
            case eDeleted:   taskState = 'D'; break;
            case eInvalid:   taskState = 'I'; break;
        }
        
        /* Print all the relevant stats for the given Task */
        LOG("%-16s%c\t%lu\t%lu\t\t%lu\n",
                taskStatusArray[task_num].pcTaskName,
                taskState,
                taskStatusArray[task_num].uxCurrentPriority,
                taskStatusArray[task_num].usStackHighWaterMark,
                taskStatusArray[task_num].xTaskNumber);
    }
    LOG("\n");
}

/* 
 * Function: Monitor_initRuntimeCounter
 * 
 * Description: Function provided for portCONFIGURE_TIMER_FOR_RUN_TIME_STATS macro in FreeConfig.h
 * Gets and saves the initial value of the System Timer for reference as the 'start' time.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 *
 */
void Monitor_initRuntimeCounter(void) 
{
    /* Get the start value of the timer */
    stats_start_time = get_absolute_time(); //returns the absolute time (now) of Pico's System Timer (hardware timer)
}

/* 
 * Function: Monitor_getRuntimeCounter
 * 
 * Description: Function provided for portGET_RUN_TIME_COUNTER_VALUE macro in FreeConfig.h
 * Returns the time since start based on the current value of System Timer and the 'start' value
 * Parameters:
 *   - none
 * 
 * Returns: unsigned long (microseconds since start)
 *
 */
unsigned long Monitor_getRuntimeCounter(void) 
{
    absolute_time_t current_time = get_absolute_time(); //returns the absolute time (now) of Pico's System Timer (hardware timer)
    int64_t time_diff_us = absolute_time_diff_us(stats_start_time, current_time);

    if (time_diff_us < 0) 
    {
        time_diff_us = 0; //if the time difference is negative, return 0 (should never happen but just in case)
    }

    if (time_diff_us > ULONG_MAX)
    {
        time_diff_us = ULONG_MAX; //if the time difference is too large, return the maximum value of an unsigned long
    }
    
    return (unsigned long)time_diff_us;
}

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

