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
#include "semphr.h"              /* xSemaphoreGetMutexHolder (DEBUG: CYW43 lock holder) */

/* SDK includes */
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "hardware/regs/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "pico/cyw43_arch.h"           /* DEBUG: cyw43_arch_async_context() */
#include "pico/async_context_freertos.h" /* DEBUG: async_context_freertos_t.lock_mutex */

/* OS includes */
#include "OS_manager.h"          /* MAX_NUM_OF_TASKS */
#include "WatchdogSupervisor.h"

/* Misc includes */
#include "Common.h"              /* LOG, CriticalErrorHandler, module/error IDs, NON_BLOCKING */
#include "FaultHandler.h"        /* FaultHandler_RecordWatchdogStall */

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* Hardware watchdog timeout. Maximum ~8.3 s on RP2350. */
#define WATCHDOG_TIMEOUT_MS 		((uint32_t)2000)

/* How long the monitored task's heartbeat may stop advancing before the
 * supervisor declares a stall, dumps diagnostics and forces a reset.
 *
 * Must exceed the worst LEGAL duration of one aliveTask loop iteration, not the
 * typical one. The LED lives on the CYW43 wifi chip, so a blink is an SPI ioctl
 * with a 1 s response timeout baked into pico-sdk (CYW43_IOCTL_TIMEOUT_US),
 * preceded by a power-save wake handshake (~130 ms worst case). One iteration
 * can therefore legally take ~500 ms period + ~1.15 s blink = ~1.65 s before
 * cyw43_gpio_set returns an error and aliveTask's own consecutive-failure
 * handling engages. 4 s gives that path ~2x margin while still catching real
 * scheduler-level hangs quickly (a 1.5 s deadline was tripped by a single slow
 * wifi-chip ioctl on 2026-07). */
#define ALIVE_STALL_DEADLINE_MS 	((uint32_t)4000)

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
static void printSavedStallDump(void);
static const char *cyw43LockHolderName(void);

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/*
 * Function: cyw43LockHolderName
 *
 * Description: DEBUG helper. Names the task currently holding the CYW43
 * async_context lock - the alive canary's only indefinite block is acquiring
 * this lock (async_context_acquire_lock_blocking in aliveTask), so when the
 * canary is found Blocked, whoever holds this lock is the prime suspect.
 *
 * cyw43_arch_async_context() returns the async_context_t base, which is the
 * first member of the async_context_freertos_t actually in use (the project
 * builds with pico_cyw43_arch_lwip_sys_freertos), so we can recover the
 * SemaphoreHandle_t lock_mutex and ask FreeRTOS who owns it.
 *
 * Returns: task name of the holder, or "(unheld)"/"(n/a)" - never NULL.
 */
static const char *cyw43LockHolderName(void)
{
	async_context_t *ctx = cyw43_arch_async_context();
	if (ctx == NULL)
	{
		return "(n/a)";
	}

	SemaphoreHandle_t lock = ((async_context_freertos_t *)ctx)->lock_mutex;
	if (lock == NULL)
	{
		return "(n/a)";
	}

	TaskHandle_t holder = xSemaphoreGetMutexHolder(lock);
	return (holder != NULL) ? pcTaskGetName(holder) : "(unheld)";
}

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
 * The dump is built into a reset-surviving buffer (lock-free, no stdio) and
 * sealed, THEN echoed live to the raw UART. Neither step touches the stdio
 * print mutex, so it cannot deadlock against a task that wedged mid-printf -
 * which is precisely the situation a stall on the CYW43/network path creates.
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

	/* DEBUG: every line below is teed into a reset-surviving RAM buffer so the
	 * boot-time park can replay this dump to a terminal attached AFTER the fact. */
	FaultHandler_StallDumpBegin();

	FaultHandler_StallDumpPrintf("\n========== WATCHDOG SUPERVISOR: STALL DETECTED ==========\n");
	FaultHandler_StallDumpPrintf("Uptime: %lu ms\n",
		   (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS));
	FaultHandler_StallDumpPrintf("Monitored task heartbeat stalled for ~%lu ms (deadline %lu ms)\n",
		   (unsigned long)stalledForMs, (unsigned long)ALIVE_STALL_DEADLINE_MS);
	FaultHandler_StallDumpPrintf("Heap: free=%u  min-ever-free=%u  largest-block=%u (bytes)\n",
		   (unsigned)heap.xAvailableHeapSpaceInBytes,
		   (unsigned)heap.xMinimumEverFreeBytesRemaining,
		   (unsigned)heap.xSizeOfLargestFreeBlockInBytes);
	FaultHandler_StallDumpPrintf("Name             St Pri StkFree  LifeMs     WinUs  Win%%\n");

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

		FaultHandler_StallDumpPrintf("%-16s %c  %2lu  %6lu  %8lu  %8lu  %3lu\n",
			   snapNow[i].pcTaskName,
			   st,
			   (unsigned long)snapNow[i].uxCurrentPriority,
			   (unsigned long)snapNow[i].usStackHighWaterMark,
			   (unsigned long)(snapNow[i].ulRunTimeCounter / 1000U),
			   (unsigned long)winUs,
			   winPct);
	}

	FaultHandler_StallDumpPrintf("Legend: St= R running / r ready / B blocked / S suspended | "
		   "StkFree=free stack words | Win%%=CPU share over the last supervisor cycle\n");
	/* DEBUG: the canary's only indefinite block is the CYW43 lock, so if it is
	 * shown Blocked above, this names the task hogging that lock - the suspect. */
	FaultHandler_StallDumpPrintf("CYW43 async_context lock currently held by: %s\n", cyw43LockHolderName());
	FaultHandler_StallDumpPrintf("No longer petting the watchdog -> board resets within ~%lu ms.\n",
		   (unsigned long)WATCHDOG_TIMEOUT_MS);
	FaultHandler_StallDumpPrintf("=========================================================\n");

	/* Seal the RAM copy so the next boot recognises it as valid, THEN echo it
	 * live over the raw UART. The seal happens first and lock-free, so even if
	 * the echo (or the impending reset) is cut short the dump still survives for
	 * the next boot's park loop to replay. */
	FaultHandler_StallDumpCommit();
	FaultHandler_StallDumpEcho();
}

/*
 * Function: printSavedStallDump
 *
 * Description: Park-loop callback (see CriticalErrorParkEx()): replays the
 * stall dump the PREVIOUS boot saved to reset-surviving RAM moments before the
 * watchdog fired, so the full task table from the moment of failure reaches a
 * terminal attached at any later time - the scratch-register breadcrumb alone
 * cannot name a CPU hog.
 *
 * Parameters: none
 *
 * Returns: void
 */
static void printSavedStallDump(void)
{
	const char *dump = FaultHandler_GetSavedStallDump();
	if (dump != NULL)
	{
		printf("---- System state captured at the moment of the stall (previous boot): ----\n");
		printf("%s", dump);
		printf("---------------------------------------------------------------------------\n");
	}
	else
	{
		printf("           (no stall dump survived in RAM - power was cycled, RAM was\n"
			   "            clobbered, or the stall was never seen by the supervisor)\n");
	}
}

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

void WatchdogSupervisor_HandleBootResetCause(void)
{
	/* A scratch register is a small, temporary storage location built into the
	 * hardware of a microcontroller peripheral (here, the watchdog). scratch[0]
	 * retains its value through a soft/watchdog reset (but is lost on power-off),
	 * so we use it as a boot-to-boot reset counter for reporting.
	 *
	 * ------------------------------------------------------------------------
	 * DIAGNOSTIC POLICY: park on the FIRST unexpected watchdog reset.
	 * ------------------------------------------------------------------------
	 * The watchdog is only ever supposed to reset us in two situations:
	 *   (a) an intentional reboot (reset_system() -> RequestReset(), e.g. OTA) -
	 *       which is tagged "clean" and is NOT a fault, or
	 *   (b) something actually went wrong (a recorded fault/stall, or an
	 *       unexplained hardware timeout).
	 *
	 * For (b) we do NOT auto-recover and we do NOT wait for repeated resets to
	 * pile up. We trap immediately and keep printing WHAT caused it, so an
	 * intermittent watchdog reset is caught on occurrence #1 instead of only
	 * after a rare "perfect storm" of back-to-back resets. This makes the board
	 * non-self-healing on purpose - it is a debugging trap. Relax it back to a
	 * counter/threshold policy once the root cause is understood. */
    if (!watchdog_enable_caused_reboot())
	{
        /* Fresh power-on or external reset - not a watchdog event. */
        watchdog_hw->scratch[0] = 0;
        return;
    }

    /* This boot followed a watchdog reboot. FaultHandler_ReportLastCrash() has
     * already run in main() and decoded the breadcrumb into RAM, so classify
     * the reset from that (the scratch breadcrumb itself is now cleared). */
    FaultHandler_ResetClass cls = FaultHandler_GetLastResetClass();

    if (cls == LAST_RESET_CLEAN)
    {
        /* Intentional reboot (e.g. applied an OTA update). Expected - carry on. */
        watchdog_hw->scratch[0] = 0;
        LOG("Clean intentional reboot - resuming normally.\n");
        return;
    }

    /* Unexpected watchdog reset -> trap. Either we have a decoded cause
     * (LAST_RESET_FAULT) or nothing recorded it (LAST_RESET_NONE), which is
     * itself a strong clue: a scheduler-level wedge / IRQs disabled / the
     * supervisor task starved - nothing in the system got to write a cause. */
    watchdog_hw->scratch[0] = watchdog_hw->scratch[0] + 1;

    const char *cause = (cls == LAST_RESET_FAULT)
        ? FaultHandler_GetLastCause()
        : "unexplained watchdog timeout - no breadcrumb "
          "(scheduler-level hang / IRQs off / supervisor task starved)";

    LOG("Unexpected watchdog reset (#%lu) - parking to preserve cause: %s\n",
        (unsigned long)watchdog_hw->scratch[0], cause);

    watchdog_disable();
    CriticalErrorParkEx(MODULE_ID_OS, ERROR_ID_WATCHDOG_RESETS, cause, printSavedStallDump);
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
	s_lastSeenHeartbeat    = s_aliveHeartbeat;
	s_lastProgressTick     = xTaskGetTickCount();
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

	uint32_t sinceProgressMs = (uint32_t)pdTICKS_TO_MS(now - s_lastProgressTick);

	if (sinceProgressMs >= ALIVE_STALL_DEADLINE_MS)
	{
		/* Stall: leave a cross-reset breadcrumb (so the cause is announced on the
		 * next boot too), print a full live dump, then stop petting.
		 *
		 * The breadcrumb carries the monitored task's state plus the CYW43 lock
		 * holder: for the common Blocked case that names the culprit in the park
		 * line without needing the live dump to have been captured. The monitored
		 * task is always the alive canary, so we do not spend a scratch slot on
		 * its (known) name. */
		eTaskState  st     = (s_monitoredTaskHandle != NULL)
			? eTaskGetState(s_monitoredTaskHandle) : eInvalid;
		const char *holder = cyw43LockHolderName();

		FaultHandler_RecordWatchdogStall((uint32_t)st, holder);
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
	/* Reset the boot-loop counter since this is an intentional reset. */
	watchdog_hw->scratch[0] = 0;

	/* Tag this reboot as clean so the next boot's HandleBootResetCause() does
	 * NOT mistake an intentional reset (e.g. applying an OTA update) for a fault
	 * and park the board. Written now, before we stop petting / reboot; the
	 * supervisor will not overwrite it because it stops running once notified. */
	FaultHandler_RecordCleanReboot();

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
