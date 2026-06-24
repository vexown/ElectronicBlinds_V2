/**
 * File: WatchdogSupervisor.c
 * Description: Implementation of the system health watchdog supervisor.
 *              See WatchdogSupervisor.h for the design rationale.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Standard includes. */
#include <stdio.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* SDK includes */
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/regs/watchdog.h"
#include "hardware/structs/watchdog.h"

/* OS includes */
#include "OS_manager.h"          /* MAX_NUM_OF_TASKS */
#include "WatchdogSupervisor.h"

/* Misc includes */
#include "Common.h"              /* LOG, CriticalErrorHandler, module/error IDs, NON_BLOCKING */
#include "FaultHandler.h"        /* FaultHandler_RecordWatchdogStall */

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* Boot-loop limit: this many watchdog resets that never reach healthy uptime
 * trips CriticalErrorHandler instead of resetting forever. */
#define MAX_WATCHDOG_RESETS 		3

/* Hardware watchdog timeout. Maximum ~8.3 s on RP2350. */
#define WATCHDOG_TIMEOUT_MS 		((uint32_t)2000)

/* How long the monitored task's heartbeat may stop advancing before the
 * supervisor declares a stall, dumps diagnostics and forces a reset. Kept just
 * under WATCHDOG_TIMEOUT_MS (and ~3 missed 500 ms aliveTask periods) so the dump
 * reflects the stalled state and matches the original "aliveTask must check in
 * within ~2 s" liveness contract. */
#define ALIVE_STALL_DEADLINE_MS 	((uint32_t)1500)

/* Once the board has run this long without a stall, the boot-loop counter
 * (scratch[0]) is cleared so isolated transient resets spread across days of
 * uptime can never accumulate to MAX_WATCHDOG_RESETS and brick the board. The
 * 3-strike rule then only trips on a true boot loop (resets that never reach
 * healthy uptime). */
#define HEALTHY_UPTIME_MS 			((uint32_t)60000)

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/* Liveness heartbeat: the monitored task bumps this once per completed loop.
 * Volatile because it is written by that task and read by the supervisor task
 * with no lock. */
static volatile uint32_t s_aliveHeartbeat = 0;

/* Handle of the monitored (low-priority canary) task, learned the first time it
 * reports liveness - avoids coupling this module to OS_manager's task handles. */
static TaskHandle_t s_monitoredTaskHandle = NULL;

/* Handle of the supervisor task itself, captured in Init() so RequestReset()
 * can notify it from another task's context. */
static TaskHandle_t s_supervisorTaskHandle = NULL;

/* Per-run supervisor state (were loop locals before this became a MainFunction). */
static uint32_t   s_lastSeenHeartbeat = 0;
static TickType_t s_lastProgressTick  = 0;
static TickType_t s_bootTick          = 0;
static bool       s_decayCleared      = false;
static bool       s_stalled           = false;

/* Snapshots for the stall dump. Kept at file scope (not on the supervisor's
 * stack) because each TaskStatus_t array is MAX_NUM_OF_TASKS entries. 'prev' is
 * refreshed every healthy supervisor cycle so that, at stall time, comparing
 * 'now' against it yields each task's CPU usage over just the last cycle - which
 * exposes a higher-priority CPU hog far more clearly than lifetime totals. */
static TaskStatus_t snapNow[MAX_NUM_OF_TASKS];
static TaskStatus_t snapPrev[MAX_NUM_OF_TASKS];
static UBaseType_t  snapPrevCount = 0;
static configRUN_TIME_COUNTER_TYPE prevTotalRunTime = 0;

/*******************************************************************************/
/*                       STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/

static void dumpSystemStateOnStall(uint32_t stalledForMs);

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/*
 * Function: dumpSystemStateOnStall
 *
 * Description: One-shot diagnostic snapshot printed the moment a stall is
 * detected, i.e. while the system is still alive and fully introspectable -
 * which is the whole advantage over reading breadcrumbs after a blind reset.
 *
 * It prints, per task: state, priority, free stack, lifetime CPU, and crucially
 * the CPU consumed over just the last supervisor cycle (the stall window). That
 * windowed share is the smoking gun:
 *   - monitored task shown as Blocked -> it was stuck on a resource (e.g. the
 *     CYW43 lock) -> the "CYW43 LED call blocks" hypothesis.
 *   - monitored task shown as Ready while another task shows a near-100%% window
 *     share -> that task is a CPU hog starving it -> the starvation hypothesis.
 *   - no hog and even this (highest-priority) task barely running -> a
 *     scheduler-level block (long critical section / IRQs off / ISR storm).
 *
 * Uses printf (not LOG) so the dump reaches the UART even if the TCP debug
 * client is gone or the network task is the thing that wedged.
 *
 * Parameters:
 *   - stalledForMs: how long the heartbeat had been stale at detection
 *
 * Returns: void
 */
static void dumpSystemStateOnStall(uint32_t stalledForMs)
{
	configRUN_TIME_COUNTER_TYPE totalNow = 0;
	UBaseType_t countNow;
	HeapStats_t heap;

	countNow = uxTaskGetSystemState(snapNow, MAX_NUM_OF_TASKS, &totalNow);
	vPortGetHeapStats(&heap);

	/* Total CPU time across all tasks over just the last supervisor cycle. */
	configRUN_TIME_COUNTER_TYPE totalWindow = totalNow - prevTotalRunTime;
	if (totalWindow == 0)
	{
		totalWindow = 1; /* guard against divide-by-zero on the very first window */
	}

	printf("\n========== WATCHDOG SUPERVISOR: STALL DETECTED ==========\n");
	printf("Monitored task heartbeat stalled for ~%lu ms (deadline %lu ms)\n",
		   (unsigned long)stalledForMs, (unsigned long)ALIVE_STALL_DEADLINE_MS);
	printf("Heap: free=%u  min-ever-free=%u  largest-block=%u (bytes)\n",
		   (unsigned)heap.xAvailableHeapSpaceInBytes,
		   (unsigned)heap.xMinimumEverFreeBytesRemaining,
		   (unsigned)heap.xSizeOfLargestFreeBlockInBytes);
	printf("Name             St Pri StkFree  LifeMs     WinUs  Win%%\n");

	for (UBaseType_t i = 0; i < countNow; i++)
	{
		char st;
		switch (snapNow[i].eCurrentState)
		{
			case eRunning:   st = 'R'; break;
			case eReady:     st = 'r'; break;
			case eBlocked:   st = 'B'; break;
			case eSuspended: st = 'S'; break;
			case eDeleted:   st = 'D'; break;
			default:         st = '?'; break;
		}

		/* Match this task in the previous snapshot (by handle) to get its CPU
		 * delta over the stall window. */
		configRUN_TIME_COUNTER_TYPE prevRun = 0;
		for (UBaseType_t j = 0; j < snapPrevCount; j++)
		{
			if (snapPrev[j].xHandle == snapNow[i].xHandle)
			{
				prevRun = snapPrev[j].ulRunTimeCounter;
				break;
			}
		}
		configRUN_TIME_COUNTER_TYPE winUs = snapNow[i].ulRunTimeCounter - prevRun;
		unsigned long winPct = (unsigned long)(((uint64_t)winUs * 100U) / totalWindow);

		printf("%-16s %c  %2lu  %6lu  %8lu  %8lu  %3lu\n",
			   snapNow[i].pcTaskName,
			   st,
			   (unsigned long)snapNow[i].uxCurrentPriority,
			   (unsigned long)snapNow[i].usStackHighWaterMark,
			   (unsigned long)(snapNow[i].ulRunTimeCounter / 1000U),
			   (unsigned long)winUs,
			   winPct);
	}

	printf("Legend: St= R running / r ready / B blocked / S suspended | "
		   "StkFree=free stack words | Win%%=CPU share over the last supervisor cycle\n");
	printf("No longer petting the watchdog -> board resets within ~%lu ms.\n",
		   (unsigned long)WATCHDOG_TIMEOUT_MS);
	printf("=========================================================\n");
}

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

void WatchdogSupervisor_HandleBootResetCause(void)
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

void WatchdogSupervisor_Enable(void)
{
	/*  Enable the watchdog timer:
	 *  	void watchdog_enable(uint32_t delay_ms, bool pause_on_debug)
	 *  	delay_ms: 		Number of milliseconds before watchdog will reboot without watchdog_update being called.
	 *  					Maximum of 8388, which is approximately 8.3 seconds
	 *  	pause_on_debug: If the watchdog should be paused when the debugger is stepping through code */
	watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

	LOG("Watchdog enabled with %lu ms timeout \n", (unsigned long)WATCHDOG_TIMEOUT_MS);
}

void WatchdogSupervisor_Init(void)
{
	s_supervisorTaskHandle = xTaskGetCurrentTaskHandle();
	s_bootTick             = xTaskGetTickCount();
	s_lastSeenHeartbeat    = s_aliveHeartbeat;
	s_lastProgressTick     = s_bootTick;
	s_decayCleared         = false;
	s_stalled              = false;

	/* Prime the 'previous' snapshot so the first stall window has a baseline. */
	snapPrevCount = uxTaskGetSystemState(snapPrev, MAX_NUM_OF_TASKS, &prevTotalRunTime);
}

void WatchdogSupervisor_MainFunction(void)
{
	TickType_t now = xTaskGetTickCount();

	/* Intentional reset from WatchdogSupervisor_RequestReset(): stop petting from
	 * now on so the watchdog reboots us. No dump/breadcrumb - this is clean. */
	if (ulTaskNotifyTake(pdTRUE, NON_BLOCKING) > 0)
	{
		s_stalled = true;
	}

	if (s_stalled)
	{
		/* Already given up on petting; just idle until the watchdog resets us
		 * (this also leaves the UART time to flush any dump printed earlier). */
		return;
	}

	/* Track monitored-task liveness. */
	uint32_t hb = s_aliveHeartbeat;
	if (hb != s_lastSeenHeartbeat)
	{
		s_lastSeenHeartbeat = hb;
		s_lastProgressTick  = now;
	}

	/* Boot-loop counter decay once we have proven healthy uptime. */
	if (!s_decayCleared &&
		(uint32_t)pdTICKS_TO_MS(now - s_bootTick) >= HEALTHY_UPTIME_MS)
	{
		watchdog_hw->scratch[0] = 0;
		s_decayCleared = true;
		LOG("Healthy for %lu ms - watchdog reset counter cleared.\n",
			(unsigned long)HEALTHY_UPTIME_MS);
	}

	uint32_t sinceProgressMs = (uint32_t)pdTICKS_TO_MS(now - s_lastProgressTick);

	if (sinceProgressMs >= ALIVE_STALL_DEADLINE_MS)
	{
		/* Stall: leave a cross-reset breadcrumb (so the cause is announced on the
		 * next boot too), print a full live dump, then stop petting. */
		eTaskState  st   = (s_monitoredTaskHandle != NULL)
			? eTaskGetState(s_monitoredTaskHandle) : eInvalid;
		const char *name = (s_monitoredTaskHandle != NULL)
			? pcTaskGetName(s_monitoredTaskHandle) : "?";

		FaultHandler_RecordWatchdogStall(name, (uint32_t)st);
		dumpSystemStateOnStall(sinceProgressMs);
		s_stalled = true;
	}
	else
	{
		/* Healthy: pet the watchdog and roll the snapshot so the next cycle's
		 * stall window has a fresh baseline to diff against. */
		watchdog_update();
		snapPrevCount = uxTaskGetSystemState(snapPrev, MAX_NUM_OF_TASKS, &prevTotalRunTime);
	}
}

void WatchdogSupervisor_ReportAlive(void)
{
	/* Learn the canary's handle from its first check-in (avoids coupling to
	 * OS_manager's task handles), then record the heartbeat. */
	if (s_monitoredTaskHandle == NULL)
	{
		s_monitoredTaskHandle = xTaskGetCurrentTaskHandle();
	}
	s_aliveHeartbeat++;
}

void WatchdogSupervisor_RequestReset(void)
{
	/* Reset the boot-loop counter since this is an intentional reset (it will
	 * still be incremented once on the next boot but won't stack up). */
	watchdog_hw->scratch[0] = 0;

	if ((watchdog_hw->ctrl & WATCHDOG_CTRL_ENABLE_BITS) && (s_supervisorTaskHandle != NULL))
	{
		/* Tell the supervisor to stop petting the watchdog, causing a reset. */
		xTaskNotifyGive(s_supervisorTaskHandle);
	}
	else
	{
		/* Watchdog was supposed to be enabled but isn't. Trigger a direct reset. */
		LOG("Watchdog not enabled, triggering direct reset.\n");
		watchdog_reboot(0, 0, 10); /* Reboot after 10ms */
	}
}
