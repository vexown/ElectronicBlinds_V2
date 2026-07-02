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

/* WiFi includes */
#include "WiFi_UDP.h"
#include "WiFi_TCP.h"
#include "WiFi_Common.h"

/* Monitor includes */
#include "Monitor_Common.h"

/* OS includes */
#include "OS_manager.h"
#include "WatchdogSupervisor.h"

/* Flash includes */
#include "flash_layout.h"
#include "flash_operations.h"
#include "metadata.h"
#include "fw_version.h"

/* Misc includes */
#include "Common.h"
#include "FaultInject.h"

/* Blinds includes */
#include "blinds_controller.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/* Priorities for the tasks (bigger number = higher prio) */
#define CYW43_INIT_TASK_PRIORITY        	(configMAX_PRIORITIES - 1)  // Highest
#define WATCHDOG_SUPERVISOR_TASK_PRIORITY	(configMAX_PRIORITIES - 2)  // Above every application task on purpose (see below)
#define ALIVE_TASK_PRIORITY             	(tskIDLE_PRIORITY + 1) 		// Lowest (but still higher than the idle task)
#define MONITOR_TASK_PRIORITY				(tskIDLE_PRIORITY + 2)
#define BLINDS_TASK_PRIORITY				(tskIDLE_PRIORITY + 2)
#define NETWORK_TASK_PRIORITY 				(tskIDLE_PRIORITY + 3)

/* Context for time-related macros, we use these two main helper macros from FreeRTOS (configTICK_RATE_HZ is defined in FreeRTOSConfig.h)
	- pdMS_TO_TICKS(TimeInMs):     (TimeInMs * configTICK_RATE_HZ) / 1000
	- pdTICKS_TO_MS(TimeInTicks):  (TimeInTicks * 1000 ) / (configTICK_RATE_HZ)
*/
/* Task periods */
#define ALIVE_TASK_PERIOD_TICKS			    pdMS_TO_TICKS(500)  //500ms
#define NETWORK_TASK_PERIOD_TICKS			pdMS_TO_TICKS(200)  //200ms
#define WIFI_RETRY_DELAY_TICKS				pdMS_TO_TICKS(30000) //30s
#define MONITOR_TASK_PERIOD_TICKS			pdMS_TO_TICKS(11000) //11s
#define BLINDS_TASK_PERIOD_TICKS			pdMS_TO_TICKS(BLINDS_TASK_PERIOD_MS)
#define WATCHDOG_SUPERVISOR_PERIOD_TICKS	pdMS_TO_TICKS(250)  //250ms - pets the watchdog ~8x per timeout window

/* Stack sizes - This parameter is in WORDS (on Pico W: 1 word = 32bit = 4bytes) */ 
#define STACK_1024_BYTES					(configSTACK_DEPTH_TYPE)(256) 
#define STACK_2048_BYTES					(configSTACK_DEPTH_TYPE)(512) 
#define STACK_4096_BYTES					(configSTACK_DEPTH_TYPE)(1024)
#define STACK_8192_BYTES					(configSTACK_DEPTH_TYPE)(2048)
#define STACK_16384_BYTES					(configSTACK_DEPTH_TYPE)(4096)

/* Watchdog tunables (timeout, boot-loop limit, stall deadline, healthy-uptime
 * decay) live with the rest of the watchdog logic in WatchdogSupervisor.c. The
 * task priority and period above are scheduling policy, so they stay here with
 * the other tasks' priorities and periods. */

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                       STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/
#if (WATCHDOG_ENABLED == ON)
static void watchdogSupervisorTask(__unused void *taskParams);
#endif
/***************************** Tasks declarations ******************************/
static void aliveTask(__unused void *taskParams);
static void cyw43initTask(__unused void *taskParams);
static void networkTask(__unused void *taskParams);
static void monitorTask(__unused void *taskParams);
static void blindsTask(__unused void *taskParams);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/*******************************************************************************/
/*                             GLOBAL VARIABLES                                */
/*******************************************************************************/
TaskHandle_t monitorTaskHandle = NULL;
TaskHandle_t aliveTaskHandle = NULL;

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
	/* Check the reset cause and enforce the boot-loop limit before anything else. */
	WatchdogSupervisor_HandleBootResetCause();
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
	taskCreationStatus[4] = xTaskCreate( blindsTask, "Blinds", STACK_2048_BYTES, NULL, BLINDS_TASK_PRIORITY, NULL);

#if (WATCHDOG_ENABLED == ON)
	/* Create the watchdog supervisor. It owns the watchdog pet (moved out of the
	 * lowest-priority aliveTask, where the pet sat behind a blocking CYW43 call)
	 * and runs at high priority so a CPU-hogging task cannot starve the pet. The
	 * supervisor captures its own handle internally, so none is needed here. */
	if (xTaskCreate(watchdogSupervisorTask, "WdSupervisor", STACK_2048_BYTES, NULL,
					WATCHDOG_SUPERVISOR_TASK_PRIORITY, NULL) != pdPASS)
	{
		CriticalErrorHandler(MODULE_ID_OS, ERROR_ID_TASK_FAILED_TO_CREATE);
	}

	/* Arm the hardware watchdog now that the supervisor exists to pet it. */
	WatchdogSupervisor_Enable();
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

    /* Report the version compiled into this image (the source of truth). */
    LOG("Current firmware version: %s \n", FW_VERSION_STR);

    /* Keep the boot metadata in sync with the running image so the field stays
     * meaningful for the bootloader (future anti-rollback). Only rewrite flash
     * when it actually differs — e.g. on the first boot after an OTA update —
     * to avoid needless flash wear on every reset. */
    if (check_current_fw_version() != FW_VERSION)
    {
        boot_metadata_t meta;
        if (read_metadata_from_flash(&meta))
        {
            meta.version = FW_VERSION;
            if (write_metadata_to_flash(&meta))
            {
                LOG("Boot metadata version synced to %s \n", FW_VERSION_STR);
            }
            else
            {
                LOG("Failed to sync boot metadata version \n");
            }
        }
    }

	/* Debug-only: spawn the fault-injection task if FAULT_INJECT_KIND != NONE.
	 * Compiled to a no-op in production builds. */
	FaultInject_Start();

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
 * Description: Reset the system cleanly. Delegates to the watchdog supervisor,
 * which stops petting the watchdog so the hardware reboots us; falls back to a
 * direct reboot when the watchdog feature is disabled.
 *
 * Parameters:
 *   - none
 *
 * Returns: void
 */
void reset_system(void)
{
#if (WATCHDOG_ENABLED == ON)
    /* Ask the supervisor to stop petting -> watchdog reboots the board. */
    WatchdogSupervisor_RequestReset();
#else
    /* Watchdog is disabled in configuration, trigger a direct reset */
    LOG("Watchdog disabled by configuration, triggering direct reset.\n");
    watchdog_reboot(0, 0, 10); /* Reboot after 10ms */
#endif

    /* Wait for the reset to take effect */
    while(1)
    {
        vTaskDelay(portMAX_DELAY);
    }
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
	setupWifiAccessPoint();
#else
	while (connectToWifi() == false)
	{
		LOG("Wi-Fi connect failed, retrying in 30s...\n");
		vTaskDelay(WIFI_RETRY_DELAY_TICKS);
	}
#endif

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

		/* DEBUG: time the two halves of the "blink" - waiting for the CYW43 lock
		 * and the cyw43_gpio_set ioctl itself. Remove together with the DEBUG
		 * watchdog-trap commits. */
		uint64_t dbgT0Us = time_us_64();

		/* Acquire the lock */
		async_context_acquire_lock_blocking(context);

		uint64_t dbgT1Us = time_us_64(); /* DEBUG */

		/* Set the state of the LED */
		int ret = cyw43_gpio_set(&cyw43_state, CYW43_WL_GPIO_LED_PIN, state_LED);

		/* Release the lock */
		async_context_release_lock(context);

		/* DEBUG: blink timing stats. Immediate warning for any suspiciously slow
		 * blink, plus a min/avg/max summary once a minute. */
		{
			uint64_t dbgT2Us = time_us_64();
			uint32_t dbgLockUs = (uint32_t)(dbgT1Us - dbgT0Us);
			uint32_t dbgGpioUs = (uint32_t)(dbgT2Us - dbgT1Us);

			static uint32_t dbgCount = 0;
			static uint32_t dbgLockMaxUs = 0;
			static uint32_t dbgGpioMinUs = UINT32_MAX;
			static uint32_t dbgGpioMaxUs = 0;
			static uint64_t dbgGpioSumUs = 0;
			static uint32_t dbgBootMaxTotalUs = 0;

			if (dbgLockUs > dbgLockMaxUs) { dbgLockMaxUs = dbgLockUs; }
			if (dbgGpioUs < dbgGpioMinUs) { dbgGpioMinUs = dbgGpioUs; }
			if (dbgGpioUs > dbgGpioMaxUs) { dbgGpioMaxUs = dbgGpioUs; }
			dbgGpioSumUs += dbgGpioUs;
			dbgCount++;

			uint32_t dbgTotalUs = dbgLockUs + dbgGpioUs;
			if (dbgTotalUs > dbgBootMaxTotalUs) { dbgBootMaxTotalUs = dbgTotalUs; }

			if (dbgTotalUs > 50000U) /* > 50 ms: log it the moment it happens */
			{
				LOG("[DEBUG] SLOW BLINK: lock wait %lu us, gpio_set %lu us\n",
					(unsigned long)dbgLockUs, (unsigned long)dbgGpioUs);
			}

			if (dbgCount >= 120U) /* ~1 min at the 500 ms period */
			{
				LOG("[DEBUG] blink timing (last %lu): gpio_set min/avg/max = %lu/%lu/%lu us, lock wait max = %lu us, worst blink since boot = %lu us\n",
					(unsigned long)dbgCount,
					(unsigned long)dbgGpioMinUs,
					(unsigned long)(dbgGpioSumUs / dbgCount),
					(unsigned long)dbgGpioMaxUs,
					(unsigned long)dbgLockMaxUs,
					(unsigned long)dbgBootMaxTotalUs);
				dbgCount = 0;
				dbgLockMaxUs = 0;
				dbgGpioMinUs = UINT32_MAX;
				dbgGpioMaxUs = 0;
				dbgGpioSumUs = 0;
			}
		}

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
		/* Signal liveness to the watchdog supervisor. This sits at the very end of
		 * the loop, AFTER the (potentially blocking) CYW43 LED section, so the
		 * heartbeat only advances on a fully completed iteration. If aliveTask
		 * blocks inside the loop or is starved of CPU, the heartbeat goes stale and
		 * the supervisor catches it. The watchdog pet itself now lives in the
		 * supervisor, decoupled from this lowest-priority task and from WiFi. */
		WatchdogSupervisor_ReportAlive();
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

/*
 * Function: blindsTask
 *
 * Description: Periodic task that manages the window blind controller.
 *              Initialises all peripherals (H-bridge, debouncer, DS3231 RTC)
 *              then runs the blinds state machine every BLINDS_TASK_PERIOD_MS.
 *
 * Parameters:
 *   - taskParams (not used)
 *
 * Returns: void (infinite loop)
 */
static void blindsTask(__unused void *taskParams)
{
	/*******************************************************************************/
	/*                          Task Initialization Code                           */
	/*******************************************************************************/
	TickType_t xLastWakeTime;

	Blinds_Status status = Blinds_Init();
	if (status != BLINDS_OK)
	{
		LOG("Blinds init failed (status=%d). Task halted.\n", (int)status);
		vTaskDelete(NULL);
		return;
	}

	/* Initialize xLastWakeTime - this only needs to be done once. */
	xLastWakeTime = xTaskGetTickCount();

	/*******************************************************************************/
	/*                               Task Loop Code                                */
	/*******************************************************************************/
	for( ;; )
	{
		vTaskDelayUntil(&xLastWakeTime, BLINDS_TASK_PERIOD_TICKS);
		Blinds_MainFunction();
	}
}

#if (WATCHDOG_ENABLED == ON)
/*
 * Function: watchdogSupervisorTask
 *
 * Description: Thin task wrapper around the WatchdogSupervisor component. Like
 * the other tasks here it only orchestrates - it initialises the supervisor and
 * then calls its main function once per period. All the watchdog logic (petting,
 * stall detection, the diagnostic dump, boot-loop counter decay) lives in
 * WatchdogSupervisor.c.
 *
 * Parameters:
 *   - taskParams (not used)
 *
 * Returns: void (infinite loop)
 */
static void watchdogSupervisorTask(__unused void *taskParams)
{
	TickType_t xLastWakeTime;

	WatchdogSupervisor_Init();

	xLastWakeTime = xTaskGetTickCount();
	for ( ;; )
	{
		vTaskDelayUntil(&xLastWakeTime, WATCHDOG_SUPERVISOR_PERIOD_TICKS);
		WatchdogSupervisor_MainFunction();
	}
}
#endif /* WATCHDOG_ENABLED == ON */

