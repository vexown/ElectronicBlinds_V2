/**
 * File: OS_hooks.c
 * Description: File containing definitions of the hooks used by the OS
 */

/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: vApplicationMallocFailedHook
 * 
 * Description: Hook called if a call to pvPortMalloc() fails because there is insufficient
 *  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
 *  internally by FreeRTOS API functions that create tasks, queues, software
 *  timers, and semaphores.  The size of the FreeRTOS heap is set by the
 *  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
void vApplicationMallocFailedHook(void)
{
    /* Force an assertion failure by passing a NULL pointer to the configASSERT macro, triggering the error handling routine. */
    configASSERT(( volatile void * ) NULL);
}

/* 
 * Function: vApplicationStackOverflowHook
 * 
 * Description: Run time stack overflow checking is performed if
 *  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook
 *  function is called if a stack overflow is detected. 
 * 
 * Parameters:
 *   - pxTask: handle of the task in which stack overflow happened
 *   - pcTaskName: task ASCII identifier
 * 
 * Returns: void
 */
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    /* cast to void to aVOID (heh) compiler warnings */ 
    (void)pxTask;
        
    printf("Stack Overflow in task: %s\n", pcTaskName);

    /* Force an assertion failure by passing a NULL pointer to the configASSERT macro, triggering the error handling routine. */
    configASSERT(( volatile void * ) NULL);
}


/* 
 * Function: vApplicationIdleHook
 * 
 * Description: The Idle Hook function is called repeatedly by the idle task, which is 
 *   the lowest priority task in the system. This function can be used to 
 *   perform low-priority background tasks when the CPU has no other work 
 *   to do. It must not attempt to block or call any API functions that could 
 *   block because the idle task needs to run whenever no other tasks are 
 *   ready to execute.
 *
 *   Typical uses for the idle hook might include monitoring system status, 
 *   performing background tasks, or placing the CPU into a low-power mode 
 *   to conserve energy when idle. If not needed, this function can be left 
 *   empty.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
void vApplicationIdleHook(void)
{

}

/* 
 * Function: vApplicationTickHook
 * 
 * Description: The Tick Hook function is called by each tick interrupt.
 *   This function allows you to execute code at the frequency
 *   of the system tick (as configured by configTICK_RATE_HZ in
 *   FreeRTOSConfig.h).
 *
 *   You can use this function to perform periodic operations,
 *   such as monitoring system status, toggling LEDs, or
 *   incrementing software timers. Since this function is called
 *   very frequently, it is important to keep the code inside it
 *   efficient and quick to execute to avoid negatively impacting
 *   the real-time performance of the system.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
void vApplicationTickHook(void)
{

}
