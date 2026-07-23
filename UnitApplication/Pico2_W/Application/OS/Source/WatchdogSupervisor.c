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

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
/* Diagnostic build only - see WATCHDOG_STALL_DIAGNOSTICS in the top CMakeLists. */
#include <stdarg.h>
#include "hardware/uart.h"       /* uart_putc_raw - lock-free dump output */
#include "pico/cyw43_arch.h"     /* cyw43_state, cyw43_ll_has_work, host wake pin */
#include "h_bridge_controller.h" /* HBridge_Stop - motors off before freezing forever */
#endif

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* Boot-loop limit: this many watchdog resets that never reach healthy uptime
 * trips CriticalErrorHandler instead of resetting forever. */
#define MAX_WATCHDOG_RESETS 		3

/* Once the board has run this long without a stall, the boot-loop counter
 * (scratch[0]) is cleared so isolated transient resets spread across days of
 * uptime can never accumulate to MAX_WATCHDOG_RESETS and brick the board. The
 * 3-strike rule then only trips on a true boot loop (resets that never reach
 * healthy uptime). */
#define HEALTHY_UPTIME_MS 			((uint32_t)60000)

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

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
/* How often the frozen board re-dumps its state, in supervisor ticks. */
#define FREEZE_REDUMP_TICKS 		((uint32_t)20)   /* 20 * 250 ms = ~5 s */

/* Longest single line the lock-free dump can emit (task rows are ~80 chars, the
 * CYW43 probe lines ~130). Longer lines are truncated, never overflowed. */
#define DIAG_LINE_MAX 				((size_t)256)
#endif

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

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
/* Snapshots for the stall dump. Kept at file scope (not on the supervisor's
 * stack) because each TaskStatus_t array is MAX_NUM_OF_TASKS entries. 'prev' is
 * refreshed every healthy supervisor cycle so that, at stall time, comparing
 * 'now' against it yields each task's CPU usage over just the last cycle - which
 * exposes a higher-priority CPU hog far more clearly than lifetime totals. */
static TaskStatus_t snapNow[MAX_NUM_OF_TASKS];
static TaskStatus_t snapPrev[MAX_NUM_OF_TASKS];
static UBaseType_t  snapPrevCount = 0;
static configRUN_TIME_COUNTER_TYPE prevTotalRunTime = 0;

/* How many stall dumps have been emitted this power cycle. >1 means the freeze
 * loop is reprinting; comparing consecutive dumps shows whether the wedge is
 * static or the system is still churning. */
static uint32_t s_dumpCount = 0;
#endif

/*******************************************************************************/
/*                       STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
static void diagPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void dumpSystemStateOnStall(uint32_t stalledForMs);
static void dumpCyw43Probe(void);
static void freezeForInspection(void);
#endif

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
/*
 * Function: diagPrintf
 *
 * Description: Emits one formatted line straight to the UART, taking no locks.
 *
 * Deliberately not printf(). A stall frequently means some task is wedged while
 * holding the pico-sdk stdio print mutex - the network task logs constantly and
 * is a prime candidate to be stuck mid-print. printf() here would block the
 * supervisor on that very mutex and the dump would never appear. vsnprintf()
 * only formats into a local buffer and uart_putc_raw() is a bare register write,
 * so neither can block. stdio on this board is UART with USB disabled, so this
 * reaches the same port a terminal is attached to.
 *
 * Parameters:
 *   - fmt, ...: printf-style format and arguments
 *
 * Returns: void
 */
static void diagPrintf(const char *fmt, ...)
{
	char    line[DIAG_LINE_MAX];
	va_list args;

	va_start(args, fmt);
	int written = vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);

	if (written <= 0)
	{
		return;
	}

	/* vsnprintf returns what it WOULD have written; clamp on truncation. */
	size_t len = ((size_t)written < sizeof(line)) ? (size_t)written : (sizeof(line) - 1U);
	for (size_t i = 0; i < len; i++)
	{
		uart_putc_raw(uart_default, line[i]);
	}
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
 * Output goes through diagPrintf(), which takes no locks - see the note there.
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

	s_dumpCount++;

	diagPrintf("\n===== WATCHDOG SUPERVISOR: STALL DETECTED (dump #%lu) =====\n",
		   (unsigned long)s_dumpCount);
	diagPrintf("Uptime: %lu ms\n",
		   (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS));
	diagPrintf("Monitored task heartbeat stalled for ~%lu ms (deadline %lu ms)\n",
		   (unsigned long)stalledForMs, (unsigned long)ALIVE_STALL_DEADLINE_MS);
	diagPrintf("Heap: free=%u  min-ever-free=%u  largest-block=%u (bytes)\n",
		   (unsigned)heap.xAvailableHeapSpaceInBytes,
		   (unsigned)heap.xMinimumEverFreeBytesRemaining,
		   (unsigned)heap.xSizeOfLargestFreeBlockInBytes);
	diagPrintf("Name             St Pri StkFree  LifeMs     WinUs  Win%%\n");

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
		diagPrintf("%-16s %c  %2lu  %6lu  %8llu  %8llu  %3lu\n",
			   snapNow[i].pcTaskName,
			   st,
			   (unsigned long)snapNow[i].uxCurrentPriority,
			   (unsigned long)snapNow[i].usStackHighWaterMark,
			   (unsigned long long)(snapNow[i].ulRunTimeCounter / 1000U),
			   (unsigned long long)winUs,
			   winPct);
	}

	diagPrintf("Legend: St= R running / r ready / B blocked / S suspended | "
		   "StkFree=free stack words | Win%%=CPU share over the last supervisor cycle\n");

	/* Bus-level probe - explains WHY the ioctls are timing out, which the task
	 * table above cannot show (it only names whoever queued on the CYW43 lock,
	 * and that is always a symptom rather than the cause). */
	dumpCyw43Probe();

	diagPrintf("FROZEN for inspection - watchdog still being petted, NO reset.\n"
		   "This dump repeats every ~%lu ms. Power-cycle to recover.\n",
		   (unsigned long)(FREEZE_REDUMP_TICKS * 250U));
	diagPrintf("=========================================================\n");
}

/*
 * Function: dumpCyw43Probe
 *
 * Description: Interrogates the CYW43 wifi chip's bus interface at stall time to
 * explain WHY every ioctl is timing out.
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

	diagPrintf("--- CYW43 bus probe ---\n");
	diagPrintf("WL_HOST_WAKE(GP%u): first=%d last=%d high=%lu/%lu samples, %lu transitions over %lu ms\n",
		   (unsigned)CYW43_PIN_WL_HOST_WAKE, first, last,
		   (unsigned long)highs, (unsigned long)samples,
		   (unsigned long)transitions,
		   (unsigned long)(sampleWindowUs / 1000U));
	diagPrintf("cyw43_ll_has_work: %s\n",
		   cyw43_ll_has_work(&cyw43_state.cyw43_ll) ? "YES (chip wants service)" : "no (chip silent)");

	/* Driver-level state, read without the lock - tells us whether the driver
	 * still believes it is associated while the chip is unreachable. join_state
	 * decodes via WIFI_JOIN_STATE_* in cyw43_ctrl.c: 0x0e01 (ACTIVE|AUTH|LINK|
	 * KEYED) is fully joined; 0x601 means the WPA key exchange never completed. */
	diagPrintf("cyw43_state: initted=%d itf_state=0x%02x join_state=0x%lx scan_state=0x%lx\n",
		   (int)cyw43_state.initted,
		   (unsigned)cyw43_state.itf_state,
		   (unsigned long)cyw43_state.wifi_join_state,
		   (unsigned long)cyw43_state.wifi_scan_state);
	diagPrintf("cyw43_state pending: disassoc=%d rejoin=%d rejoin_wpa=%d\n",
		   (int)cyw43_state.pend_disassoc,
		   (int)cyw43_state.pend_rejoin,
		   (int)cyw43_state.pend_rejoin_wpa);
	diagPrintf("Read as: pin never high -> chip hung/asleep (chip-side). "
		   "pin high or toggling -> host RX path not draining it (host-side).\n");
}

/*
 * Function: freezeForInspection
 *
 * Description: Never returns. Instead of letting the board reset, hold it in the
 * stalled state indefinitely and keep re-dumping, so the evidence waits for the
 * observer instead of the other way round.
 *
 * Why this beats capturing across a reset: a reset destroys the live state, so
 * the dump would have to survive in RAM the bootloader may reuse AND a terminal
 * would have to be recording at the exact instant of the crash. Frozen, the
 * board just keeps reprinting the full task table until someone connects.
 *
 * Mechanics:
 *   - the hardware watchdog keeps being petted here, so it never fires;
 *   - vTaskDelay (not busy-wait) is used deliberately: the rest of the system
 *     keeps running exactly as it was, so the wedge stays genuine and the Win%
 *     column keeps showing who is really burning CPU;
 *   - each redump re-samples live state and writes it over the raw, lock-free
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
#endif /* WATCHDOG_STALL_DIAGNOSTICS */

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

void WatchdogSupervisor_HandleBootResetCause(void)
{
	/* A scratch register is a small, temporary storage location built into the
	 * hardware of a microcontroller peripheral (here, the watchdog). scratch[0]
	 * retains its value through a soft/watchdog reset (but is lost on power-off),
	 * so we use it as a boot-to-boot reset counter.
	 *
	 * Policy: reset and continue. A single watchdog reset is a recovery, not a
	 * fault to trap on - the board must get itself working again unattended.
	 * Only a genuine boot loop (MAX_WATCHDOG_RESETS resets that never reach
	 * HEALTHY_UPTIME_MS) parks the board, because at that point resetting again
	 * clearly is not helping. The cause is logged either way.
	 *
	 * To investigate a stall rather than recover from it, build with
	 * -DWATCHDOG_STALL_DIAGNOSTICS=ON: the supervisor then freezes and dumps
	 * instead of ever reaching a reset. */
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

    /* Unexpected watchdog reset. Either we have a decoded cause
     * (LAST_RESET_FAULT) or nothing recorded it (LAST_RESET_NONE), which is
     * itself a strong clue: a scheduler-level wedge / IRQs disabled / the
     * supervisor task starved - nothing in the system got to write a cause. */
    watchdog_hw->scratch[0] = watchdog_hw->scratch[0] + 1;
    uint32_t count = watchdog_hw->scratch[0];

    const char *cause = (cls == LAST_RESET_FAULT)
        ? FaultHandler_GetLastCause()
        : "unexplained watchdog timeout - no breadcrumb "
          "(scheduler-level hang / IRQs off / supervisor task starved)";

    LOG("Watchdog reset detected (#%lu of %d): %s\n",
        (unsigned long)count, MAX_WATCHDOG_RESETS, cause);

    if (count >= MAX_WATCHDOG_RESETS)
    {
        /* Boot loop: resetting is not recovering us. Stop and preserve why. */
        LOG("Too many watchdog resets without healthy uptime - entering error state\n");
        watchdog_disable();
        CriticalErrorParkEx(MODULE_ID_OS, ERROR_ID_WATCHDOG_RESETS, cause, NULL);
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
	s_lastSeenHeartbeat    = s_aliveHeartbeat;
	s_lastProgressTick     = xTaskGetTickCount();
	s_bootTick             = s_lastProgressTick;
	s_decayCleared         = false;
	s_stalled              = false;

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
	/* Prime the 'previous' snapshot so the first stall window has a baseline. */
	snapPrevCount = uxTaskGetSystemState(snapPrev, MAX_NUM_OF_TASKS, &prevTotalRunTime);
#endif
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

	/* Once we have run cleanly for a while, forget past watchdog resets so
	 * isolated transient faults spread over days cannot accumulate into a false
	 * boot-loop trip. */
	if (!s_decayCleared &&
		((uint32_t)pdTICKS_TO_MS(now - s_bootTick) >= HEALTHY_UPTIME_MS))
	{
		watchdog_hw->scratch[0] = 0;
		s_decayCleared = true;
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
		/* Stall: leave a cross-reset breadcrumb so the cause is announced on the
		 * next boot, then stop petting and let the watchdog reset us. The
		 * monitored task is always the alive canary, so we do not spend a scratch
		 * slot on its (known) name - just its state at detection. */
		eTaskState st = (s_monitoredTaskHandle != NULL)
			? eTaskGetState(s_monitoredTaskHandle) : eInvalid;

		FaultHandler_RecordWatchdogStall((uint32_t)st);
		s_stalled = true;

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
		/* Diagnostic build: never reach the reset. Dump the live state and hold
		 * the board here, re-dumping forever, so the evidence waits for whoever
		 * attaches a terminal. Never returns. */
		dumpSystemStateOnStall(sinceProgressMs);
		freezeForInspection();
#endif
	}
	else
	{
		/* Healthy: pet the watchdog. */
		watchdog_update();

#if defined(WATCHDOG_STALL_DIAGNOSTICS)
		/* Roll the snapshot so the next cycle's stall window has a fresh baseline
		 * to diff against. */
		snapPrevCount = uxTaskGetSystemState(snapPrev, MAX_NUM_OF_TASKS, &prevTotalRunTime);
#endif
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
