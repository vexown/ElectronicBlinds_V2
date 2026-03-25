/**
 * File: OS_manager.c
 * Description: High-level FreeRTOS configuration. Setup of tasks and other 
 * system-related mechanisms 
 */

/* FreeRTOS V202107.00
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
 *******************************************************************************/

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"

/* SDK includes */
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "pico/cyw43_arch.h"
#include "lwip/sockets.h"
#include "pico/async_context.h"
#include "hardware/watchdog.h"
#include "hardware/regs/watchdog.h"
#include "hardware/structs/watchdog.h"

/* WiFi includes */
#include "WiFi_UDP.h"
#include "WiFi_TCP.h"
#include "WiFi_Common.h"

/* Monitor includes */
#include "Monitor_Common.h"

/* OS includes */
#include "OS_manager.h"

/* Flash includes */
#include "flash_layout.h"
#include "flash_operations.h"
#include "metadata.h"

/* Misc includes */
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/* Priorities for the tasks (bigger number = higher prio) */
#define CYW43_INIT_TASK_PRIORITY        	(configMAX_PRIORITIES - 1)  // Highest
#define ALIVE_TASK_PRIORITY             	(tskIDLE_PRIORITY + 1) 		// Lowest (but still higher than the idle task)
#define MONITOR_TASK_PRIORITY				(tskIDLE_PRIORITY + 2)
#define NETWORK_TASK_PRIORITY 				(tskIDLE_PRIORITY + 3)

/* Context for time-related macros, we use these two main helper macros from FreeRTOS (configTICK_RATE_HZ is defined in FreeRTOSConfig.h)
	- pdMS_TO_TICKS(TimeInMs):     (TimeInMs * configTICK_RATE_HZ) / 1000
	- pdTICKS_TO_MS(TimeInTicks):  (TimeInTicks * 1000 ) / (configTICK_RATE_HZ)
*/
/* Task periods */
#define ALIVE_TASK_PERIOD_TICKS			    pdMS_TO_TICKS(500)  //500ms
#define NETWORK_TASK_PERIOD_TICKS			pdMS_TO_TICKS(200)  //200ms
#define MONITOR_TASK_PERIOD_TICKS			pdMS_TO_TICKS(11000) //11s

/* Stack sizes - This parameter is in WORDS (on Pico W: 1 word = 32bit = 4bytes) */ 
#define STACK_1024_BYTES					(configSTACK_DEPTH_TYPE)(256) 
#define STACK_2048_BYTES					(configSTACK_DEPTH_TYPE)(512) 
#define STACK_4096_BYTES					(configSTACK_DEPTH_TYPE)(1024)
#define STACK_8192_BYTES					(configSTACK_DEPTH_TYPE)(2048)
#define STACK_16384_BYTES					(configSTACK_DEPTH_TYPE)(4096)

/* Watchdog settings */
#define MAX_WATCHDOG_RESETS 				3
#define WATCHDOG_TIMEOUT_MS 				((uint32_t)2000)

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                       STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/
#if (WATCHDOG_ENABLED == ON)
static void checkResetReason(void);
#endif
/***************************** Tasks declarations ******************************/
static void aliveTask(__unused void *taskParams);
static void cyw43initTask(__unused void *taskParams);
static void networkTask(__unused void *taskParams);
static void monitorTask(__unused void *taskParams);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/*******************************************************************************/
/*                             GLOBAL VARIABLES                                */
/*******************************************************************************/
TaskHandle_t monitorTaskHandle = NULL;
TaskHandle_t aliveTaskHandle = NULL;

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

#if (WATCHDOG_ENABLED == ON)
/* 
 * Function: checkResetReason
 * 
 * Description: Check the reset cause and handle it accordingly
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
static void checkResetReason(void) 
{
	/* Check if the system was reset by the watchdog */
    if (watchdog_enable_caused_reboot()) 
	{
        /* If the system was reset by the watchdog, increment the counter in the scratch register.
         * A scratch register is a small, temporary storage location built into the hardware of a microcontroller 
         * peripheral, such as a watchdog timer, UART, or DMA controller. 
         * 
         * Here we use the property of the Scratch Register which is that they retain information through a 
         * soft reset of the chip, such as watchdog reset (they still lose their contents during a power-off though (hard reset)) */
        watchdog_hw->scratch[0] = watchdog_hw->scratch[0] + 1;
        uint32_t count = watchdog_hw->scratch[0];

        LOG("Watchdog reset detected! Count: %ld\n", count);
        
        if (count >= MAX_WATCHDOG_RESETS) 
		{
            LOG("Too many watchdog resets! Entering error state\n");
            watchdog_disable();

			CriticalErrorHandler(MODULE_ID_OS, ERROR_ID_WATCHDOG_RESETS);
        }
    } 
	else 
	{
        /* Reset not caused by watchdog, reset the counter */
        watchdog_hw->scratch[0] = 0;
    }
}
#endif

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/
/* 
 * Function: OS_start
 * 
 * Description: Global function called from main.c. Initial OS setup and starts scheduler
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
void OS_start( void )
{
	const char *rtos_type;
	BaseType_t taskCreationStatus[NUM_OF_TASKS_TO_CREATE];

#if (WATCHDOG_ENABLED == ON)
	/* Check if the system was reset by the watchdog */
	checkResetReason();
#endif

    /** Check if we're running FreeRTOS on single core or both RP2350 cores:
      * - Standard FreeRTOS is designed for single-core systems, with simpler task scheduling and communication mechanisms.
	  * - FreeRTOS SMP is an enhanced version for multi-core systems, allowing tasks to run concurrently across multiple cores */
#if (configNUMBER_OF_CORES == 2)
 	rtos_type = "FreeRTOS SMP";
    LOG("Running %s on both cores \n", rtos_type);
#else
	rtos_type = "FreeRTOS";
	LOG("Running %s on one core \n", rtos_type);
#endif

	LOG("Setting up the RTOS configuration... \n");

	/* Create the tasks */
	taskCreationStatus[0] = xTaskCreate( aliveTask,                 /* The function that implements the task. */
										"AliveLED",                 /* The text name assigned to the task - for debug only as it is not used by the kernel. */
										STACK_1024_BYTES,           /* The size of the stack to allocate to the task (in words) */
										NULL,                       /* The parameter passed to the task - not used in this case. */
										ALIVE_TASK_PRIORITY,        /* The priority assigned to the task. */
										&aliveTaskHandle );         /* The task handle, if not needed put NULL */
	taskCreationStatus[1] = xTaskCreate( networkTask, "Network", STACK_8192_BYTES, NULL, NETWORK_TASK_PRIORITY, NULL);
	taskCreationStatus[2] = xTaskCreate( monitorTask, "Monitor", STACK_2048_BYTES, NULL, MONITOR_TASK_PRIORITY, &monitorTaskHandle);
	taskCreationStatus[3] = xTaskCreate( cyw43initTask, "CYW43_Init", STACK_1024_BYTES, NULL, CYW43_INIT_TASK_PRIORITY, NULL); // Must be highest priority, will be deleted after it runs

#if (WATCHDOG_ENABLED == ON)
    /*  Enable the watchdog timer: 
	 *  	void watchdog_enable(uint32_t delay_ms, bool pause_on_debug)
	 *  	delay_ms: 		Number of milliseconds before watchdog will reboot without watchdog_update being called. 
	              		Maximum of 8388, which is approximately 8.3 seconds
     *  	pause_on_debug: If the watchdog should be paused when the debugger is stepping through code */
	watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

	LOG("Watchdog enabled with %ld ms timeout \n", WATCHDOG_TIMEOUT_MS);
#endif

	/* Check if the tasks were created successfully */
	for(uint8_t i = 0; i < NUM_OF_TASKS_TO_CREATE; i++)
	{
		if(taskCreationStatus[i] == pdPASS)
		{
			LOG("Task number %u created successfully \n", i);
		}
		else /* pdFAIL */
		{
			CriticalErrorHandler(MODULE_ID_OS, ERROR_ID_TASK_FAILED_TO_CREATE);
		}
	}

    /* Check which bank is the application running from */
    uint8_t active_bank = check_active_bank(); 
    if(active_bank == BANK_A)
    {
        LOG("Running from Bank A \n");
    }
    else if(active_bank == BANK_B)
    {
        LOG("Running from Bank B \n");
    }
    else
    {
        LOG("Invalid bank (0xFF) \n"); // we should never get here, bootloader shall not allow to boot with invalid bank in metadata
    }

    /* Check FW version */
    uint32_t current_version = check_current_fw_version();
    LOG("Current firmware version: %lu \n", current_version);

	LOG("RTOS configuration finished, starting the scheduler... \n");
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );
}

/* 
 * Function: reset_system
 * 
 * Description: Reset the system by notifying the aliveTask to stop petting the watchdog
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
void reset_system(void)
{
#if (WATCHDOG_ENABLED == ON)
    /* Reset the watchdog reset counter since this is an intentional reset (it will still be incremented once but won't stack up) */
    watchdog_hw->scratch[0] = 0;

    /* Check if the watchdog is actually enabled */
    if (watchdog_hw->ctrl & WATCHDOG_CTRL_ENABLE_BITS)
    {
        /* Tell the aliveTask to stop petting the watchdog in turn causing a reset */
        xTaskNotifyGive(aliveTaskHandle);

        /* Wait for the watchdog to reset the system */
        while(1)
        {
            vTaskDelay(portMAX_DELAY);
        }
    }
    else
    {
        /* Watchdog was supposed to be enabled but isn't. Trigger a direct reset. */
        LOG("Watchdog not enabled, triggering direct reset.\n");
        /* Parameters: pc, sp, delay_ms */
        watchdog_reboot(0, 0, 10); /* Reboot after 10ms */
        /* Wait for the reset */
        while(1)
        {
            vTaskDelay(portMAX_DELAY);
        }
    }
#else
    /* Watchdog is disabled in configuration, trigger a direct reset */
    LOG("Watchdog disabled by configuration, triggering direct reset.\n");
    watchdog_reboot(0, 0, 10); /* Reboot after 10ms */
    /* Wait for the reset */
    while(1)
    {
        vTaskDelay(portMAX_DELAY);
    }
#endif
}

/*******************************************************************************/
/*                            TASK FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: monitorTask
 * 
 * Description: Monitoring Task providing information about the system behavior and performance
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
static void monitorTask(__unused void *taskParams) 
{
	/*******************************************************************************/
	/*                          Task Initialization Code                           */
	/*******************************************************************************/
	TickType_t xLastWakeTime;

#if (MONITORING_ENABLED == ON)
    static bool monitoringEnabled = true; //start off with monitoring enabled
#else
    static bool monitoringEnabled = false; //start off with monitoring disabled
#endif

	/* Initialize xLastWakeTime - this only needs to be done once. */
	xLastWakeTime = xTaskGetTickCount();
	/*******************************************************************************/
	/*                               Task Loop Code                                */
	/*******************************************************************************/
    for( ;; )
	{
		vTaskDelayUntil(&xLastWakeTime, MONITOR_TASK_PERIOD_TICKS); // Execute periodically at consistent intervals based on a reference time

        /* Check if the monitoring task was notified to toggle the monitoring state.
            Clear the notification after receiving it. Do not block the task to wait for the notification, just check if it is available. */
        if(ulTaskNotifyTake(pdTRUE, NON_BLOCKING) > 0) // Checks task's notification count before it is cleared (pdTRUE means we clear it after checking)
        {
            /* Toggle the monitoring state */
            monitoringEnabled = !monitoringEnabled;

            if(monitoringEnabled)
            {
                LOG("Monitoring enabled \n");
            }
            else
            {
                LOG("Monitoring disabled \n");
            }
        }

        /* If monitoring is enabled, call the main function of the monitoring module */
        if(monitoringEnabled)
        {
            Monitor_MainFunction();
        }
        else
        {
            /* Do nothing, monitoring is disabled */
        }
    }
}

/* 
 * Function: networkTask
 * 
 * Description: Network task handling the Wi-Fi communication (receive/send TCP and UDP)
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
static void networkTask(__unused void *taskParams)
{
	/*******************************************************************************/
	/*                          Task Initialization Code                           */
	/*******************************************************************************/
#if (PICO_AS_ACCESS_POINT == ON)
	if(setupWifiAccessPoint() == false)
#else
	if(connectToWifi() == false)
#endif
	{
		CriticalErrorHandler(MODULE_ID_OS, ERROR_ID_WIFI_DID_NOT_CONNECT);
	}

	/*******************************************************************************/
	/*                               Task Loop Code                                */
	/*******************************************************************************/
    for( ;; )
	{
        vTaskDelay(NETWORK_TASK_PERIOD_TICKS); 

		WiFi_MainFunction();
	}

}

/* 
 * Function: aliveTask
 * 
 * Description: Task responsible for blinking the LED to indicate the system is alive
 * 				Also, it resets the watchdog timer to prevent the system from rebooting.
 * 
 * Parameters:
 *   - taskParams (not used)
 * 
 * Returns: void
 */
static void aliveTask(__unused void *taskParams)
{
	/*******************************************************************************/
	/*                          Task Initialization Code                           */
	/*******************************************************************************/
	TickType_t xLastWakeTime;

#if (ALIVE_LED_ENABLED == ON)
	static bool state_LED;
	static uint8_t consecutiveFailures = 0;
#endif

	/* Initialize xLastWakeTime - this only needs to be done once. */
	xLastWakeTime = xTaskGetTickCount();
	/*******************************************************************************/
	/*                               Task Loop Code                                */
	/*******************************************************************************/
	for( ;; )
	{
		vTaskDelayUntil(&xLastWakeTime, ALIVE_TASK_PERIOD_TICKS); /* Execute periodically at consistent intervals based on a reference time */

#if (ALIVE_LED_ENABLED == ON)
		state_LED = !state_LED; // Toggle the state of the LED
		
		/* Get the async context from the CYW43 driver. async_context is a data structure used for managing asynchronous operations 
			in a thread-safe manner. It maintains an internal event queue where asynchronous events are posted and processed. 
			In FreeRTOS where we use the pico_cyw43_arch_lwip_sys_freertos library, a dedicated task is used to process these events */
		async_context_t *context = cyw43_arch_async_context();

		/* Acquire the lock */
		async_context_acquire_lock_blocking(context);
		
		/* Set the state of the LED */
		int ret = cyw43_gpio_set(&cyw43_state, CYW43_WL_GPIO_LED_PIN, state_LED);
		
		/* Release the lock */
		async_context_release_lock(context);

		/* Check if the LED was set successfully */
		if(ret != 0)
		{
			consecutiveFailures++;
			LOG("LED failure number %d \n", consecutiveFailures);
			
			if(consecutiveFailures >= 3)
			{
				/* If the LED fails to set 3 times in a row, enter error state */
				CriticalErrorHandler(MODULE_ID_OS, ERROR_ID_LED_FAILED);
			}
		}
		else
		{
			/* Reset the counter if the LED was set successfully */
			consecutiveFailures = 0;
		}
#endif

#if (WATCHDOG_ENABLED == ON)
		/* TODO - add additional critical system checks that should be done before resetting the watchdog */

        /* Check if a reset was requested, in which case we don't pet the watchdog */
        if(ulTaskNotifyTake(pdTRUE, NON_BLOCKING) > 0) // Checks task's notification count before it is cleared (pdTRUE means we clear it after checking)
        {
            vTaskDelay(pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS + 100)); // Wait long enough to let the watchdog timeout (trigger a reset)
        }
        else
        {
            /* Pet the watchdog to keep him from barking (rebooting the system) */
            watchdog_update();
        }
#endif

	}
}

/* 
 * Function: cyw43initTask
 * 
 * Description: Task initializing the CYW43 wireless chip. Must be run first (set to highest priority).
 * 
 * Parameters:
 *   - taskParams (not used)
 * 
 * Returns: void
 */
static void cyw43initTask(__unused void *taskParams)
{
	/*******************************************************************************/
	/*                          Task Initialization Code                           */
	/*******************************************************************************/

	/* Initialize the cyw43_driver code and the lwIP stack */
	int ret_status = cyw43_arch_init();
	if(ret_status != 0)
	{
		CriticalErrorHandler(MODULE_ID_OS, ERROR_ID_CYW43_INIT_FAILED);
	}

	/* Delete the task after it runs */
	vTaskDelete(NULL);
}

