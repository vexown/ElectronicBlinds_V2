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

/* Driver includes */
#include "h_bridge_controller.h" /* HBridge_Stop - motors off before freezing forever */

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

/* DEBUG (stall hunt): when a stall is detected, FREEZE instead of resetting -
 * keep petting the watchdog forever and re-dump the live system state every
 * FREEZE_REDUMP_TICKS supervisor ticks.
 *
 * Rationale: every reset-based capture path we tried loses the evidence. The
 * dump has to survive a watchdog reset in RAM the bootloader may reuse, and a
 * terminal has to be recording at the exact moment of the reset. Staying alive
 * removes both problems - the stalled state simply sits there, reprinting, until
 * someone plugs in a UART adapter, whether that is 10 seconds or 10 hours later.
 *
 * This is safe because the supervisor runs at configMAX_PRIORITIES-2 (only the
 * timer task is above it) and, by definition, it is running - it just detected
 * the stall - so it can keep the hardware watchdog fed. Motors are stopped on
 * entry, because "never reset" means nothing else will stop them.
 *
 * Revert together with the other stall-hunt DEBUG commits once the root cause is
 * found; the production policy is reset-and-continue. */
#define FREEZE_REDUMP_TICKS 		((uint32_t)20)   /* 20 * 250 ms = ~5 s */

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

/* DEBUG (stall hunt): how many stall dumps have been emitted this power cycle.
 * >1 means the freeze park is reprinting. */
static uint32_t s_dumpCount = 0;

/*******************************************************************************/
/*                       STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/

static void dumpSystemStateOnStall(uint32_t stalledForMs);
static void dumpCyw43Probe(void);
static void freezeForInspection(void);
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

	/* Numbered so repeats are distinguishable while frozen: comparing consecutive
	 * dumps shows whether the wedge is static (same holder, no CPU moving) or the
	 * system is churning. */
	s_dumpCount++;

	FaultHandler_StallDumpPrintf("\n===== WATCHDOG SUPERVISOR: STALL DETECTED (dump #%lu) =====\n",
		   (unsigned long)s_dumpCount);
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

		/* 64-bit run-time counters (configRUN_TIME_COUNTER_TYPE): print with %llu
		 * so nothing is truncated on a long-running board. */
		FaultHandler_StallDumpPrintf("%-16s %c  %2lu  %6lu  %8llu  %8llu  %3lu\n",
			   snapNow[i].pcTaskName,
			   st,
			   (unsigned long)snapNow[i].uxCurrentPriority,
			   (unsigned long)snapNow[i].usStackHighWaterMark,
			   (unsigned long long)(snapNow[i].ulRunTimeCounter / 1000U),
			   (unsigned long long)winUs,
			   winPct);
	}

	FaultHandler_StallDumpPrintf("Legend: St= R running / r ready / B blocked / S suspended | "
		   "StkFree=free stack words | Win%%=CPU share over the last supervisor cycle\n");
	/* DEBUG: the canary's only indefinite block is the CYW43 lock, so if it is
	 * shown Blocked above, this names the task hogging that lock - the suspect. */
	FaultHandler_StallDumpPrintf("CYW43 async_context lock currently held by: %s\n", cyw43LockHolderName());

	/* DEBUG: bus-level probe - explains WHY the ioctls are timing out, which the
	 * task table above cannot show (it only ever names whoever queued on the
	 * lock, which is a symptom). */
	dumpCyw43Probe();

	FaultHandler_StallDumpPrintf("FROZEN for inspection - watchdog still being petted, NO reset.\n"
		   "This dump repeats every ~%lu ms. Power-cycle to recover.\n",
		   (unsigned long)(FREEZE_REDUMP_TICKS * 250U));
	FaultHandler_StallDumpPrintf("=========================================================\n");

	/* Seal the RAM copy so the next boot recognises it as valid, THEN echo it
	 * live over the raw UART. The seal happens first and lock-free, so even if
	 * the echo (or the impending reset) is cut short the dump still survives for
	 * the next boot's park loop to replay. */
	FaultHandler_StallDumpCommit();
	FaultHandler_StallDumpEcho();
}

/*
 * Function: dumpCyw43Probe
 *
 * Description: DEBUG (stall hunt). Interrogates the CYW43 wifi chip's bus
 * interface at stall time to explain WHY every ioctl is timing out.
 *
 * Background. The driver logs `STALL(0;204-204): timeout` - see
 * cyw43_sdpcm_send_common() in cyw43_ll.c. Decoded that is wlan_flow_control=0,
 * tx sequence=204, last granted bus credit=204. The host may only send while
 * those two differ, so we have spent every credit the chip granted and the chip
 * has granted no more. Credits are only ever refreshed by the header of a packet
 * RECEIVED from the chip, so a chip that sends nothing stalls the host forever.
 *
 * The decisive question is therefore whether the chip is still asking to be
 * serviced. cyw43_ll_sdpcm_poll_device() (cyw43_ll.c:1016) begins with:
 *
 *     if (!self->had_successful_packet && !cyw43_cb_read_host_interrupt_pin(...))
 *         return -1;
 *
 * i.e. the poll is gated entirely on the WL_HOST_WAKE pin (GP24). If that pin is
 * low the driver returns immediately WITHOUT TOUCHING THE BUS - so a full second
 * of "polling" can consist of never talking to the chip at all. Sampling the pin
 * splits the diagnosis cleanly:
 *
 *   - pin never asserts  -> the chip is not requesting service. It is hung,
 *     crashed or asleep; the host is waiting on a radio that will never speak.
 *   - pin asserted/toggling -> the chip IS asking for service and the host-side
 *     receive path is failing to drain it (bus/PIO desync, or an interrupt being
 *     cleared without the packet being consumed).
 *
 * Worth knowing: the driver's own stall recovery - the "do poke" of the SDIO
 * mailbox at cyw43_ll.c:650 - is inside `#if !CYW43_USE_SPI`, and Pico W /
 * Pico 2 W set CYW43_USE_SPI=1. On this board that poke is compiled out, so the
 * stall loop is purely passive: it polls a pin for one second and gives up. There
 * is no mechanism anywhere in that path to nudge a wedged chip.
 *
 * Safety: everything here is either a GPIO read or a plain read of a
 * cyw43_state field. Nothing acquires the CYW43 lock (the stalled task is
 * holding it) and nothing issues a bus transaction, so this cannot deadlock and
 * cannot disturb the wedged state we are trying to observe. Deliberately does
 * NOT call cyw43_wifi_link_status() etc - those take CYW43_THREAD_ENTER.
 *
 * Parameters: none
 *
 * Returns: void
 */
static void dumpCyw43Probe(void)
{
	/* Sample WL_HOST_WAKE tightly for ~50 ms. A single read could easily miss a
	 * brief service request, and "did it ever go high" is exactly the datum that
	 * decides chip-side vs host-side. Busy-sampled rather than delayed so short
	 * pulses are actually caught; 50 ms at this priority is harmless. */
	const uint64_t sampleWindowUs = 50000U;
	uint32_t samples    = 0;
	uint32_t highs      = 0;
	uint32_t transitions = 0;
	int      last       = cyw43_cb_read_host_interrupt_pin(NULL);
	int      first      = last;

	uint64_t t0 = time_us_64();
	while ((time_us_64() - t0) < sampleWindowUs)
	{
		int now = cyw43_cb_read_host_interrupt_pin(NULL);
		samples++;
		if (now) { highs++; }
		if (now != last) { transitions++; last = now; }
	}

	FaultHandler_StallDumpPrintf("--- CYW43 bus probe ---\n");
	FaultHandler_StallDumpPrintf("WL_HOST_WAKE(GP%u): first=%d last=%d high=%lu/%lu samples, %lu transitions over %lu ms\n",
		   (unsigned)CYW43_PIN_WL_HOST_WAKE, first, last,
		   (unsigned long)highs, (unsigned long)samples,
		   (unsigned long)transitions,
		   (unsigned long)(sampleWindowUs / 1000U));
	FaultHandler_StallDumpPrintf("cyw43_ll_has_work: %s\n",
		   cyw43_ll_has_work(&cyw43_state.cyw43_ll) ? "YES (chip wants service)" : "no (chip silent)");

	/* Driver-level state, read without the lock - tells us whether the driver
	 * still believes it is associated while the chip is unreachable. */
	FaultHandler_StallDumpPrintf("cyw43_state: initted=%d itf_state=0x%02x join_state=0x%lx scan_state=0x%lx\n",
		   (int)cyw43_state.initted,
		   (unsigned)cyw43_state.itf_state,
		   (unsigned long)cyw43_state.wifi_join_state,
		   (unsigned long)cyw43_state.wifi_scan_state);
	FaultHandler_StallDumpPrintf("cyw43_state pending: disassoc=%d rejoin=%d rejoin_wpa=%d\n",
		   (int)cyw43_state.pend_disassoc,
		   (int)cyw43_state.pend_rejoin,
		   (int)cyw43_state.pend_rejoin_wpa);
	FaultHandler_StallDumpPrintf("Read as: pin never high -> chip hung/asleep (chip-side). "
		   "pin high or toggling -> host RX path not draining it (host-side).\n");
}

/*
 * Function: freezeForInspection
 *
 * Description: DEBUG (stall hunt). Never returns. Instead of letting the board
 * reset, hold it in the stalled state indefinitely and keep re-dumping, so the
 * evidence waits for the observer instead of the other way round.
 *
 * Why this beats every reset-based capture we tried: a reset destroys the live
 * state, so the dump had to survive in RAM the bootloader may reuse, AND a
 * terminal had to be recording at the exact instant of the crash. Three stalls in
 * a row produced nothing usable for exactly those reasons. Frozen, the board just
 * keeps reprinting the full task table until someone connects.
 *
 * Mechanics:
 *   - the hardware watchdog keeps being petted here, so it never fires;
 *   - vTaskDelay (not busy-wait) is used deliberately: the rest of the system
 *     keeps running exactly as it was, so the wedge stays genuine and the Win%
 *     column keeps showing who is really burning CPU;
 *   - each redump re-samples live state and echoes it over the raw, lock-free
 *     UART path, so it cannot deadlock on the stdio mutex;
 *   - the CPU window in every redump is measured against the LAST HEALTHY cycle's
 *     snapshot, so Win% accumulates over the whole frozen period - a hog stands
 *     out more with every repeat.
 *
 * Motors are stopped first: "never reset" also means nothing else will ever stop
 * them, and a blind driving into its end stop for hours is a real hazard.
 *
 * Parameters: none
 *
 * Returns: never
 */
static void freezeForInspection(void)
{
	(void)HBridge_Stop();

	for ( ;; )
	{
		for (uint32_t i = 0; i < FREEZE_REDUMP_TICKS; i++)
		{
			watchdog_update();
			vTaskDelay(pdMS_TO_TICKS(250));
		}

		uint32_t stalledForMs =
			(uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - s_lastProgressTick);
		dumpSystemStateOnStall(stalledForMs);
	}
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
		 * next boot too), print a full live dump, then freeze for inspection.
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

		/* DEBUG (stall hunt): do NOT stop petting - freeze here instead and keep
		 * reprinting the live state forever. Never returns. The breadcrumb above
		 * is still recorded so that an unrelated reset (brownout, power blip)
		 * would still be explained on the next boot. */
		freezeForInspection();
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
