/**
 * File: WatchdogSupervisor.h
 * Description: System health watchdog supervisor.
 *
 * This component owns everything to do with the hardware watchdog: the
 * boot-time reset-cause / boot-loop check, arming the timer, and the runtime
 * supervisor that actually pets it. OS_manager only orchestrates - it creates a
 * thin task that calls into here and wires up the few entry points below.
 *
 * Design rationale (why the pet is NOT in the lowest-priority task anymore):
 *
 *   The liveness *detector* is still a low-priority canary: the monitored task
 *   (aliveTask) calls WatchdogSupervisor_ReportAlive() once per completed loop,
 *   and the supervisor only pets the hardware watchdog while that heartbeat
 *   keeps advancing. So "did the bottom task get to run?" remains the health
 *   criterion - if a CPU hog or a blocking call starves it, the heartbeat stops
 *   and the board is reset.
 *
 *   What changed is *who pets and observes*. A starved low-priority task cannot
 *   run code, so it cannot print why it was starved - which is exactly why the
 *   original freezes produced no logs. The supervisor therefore runs at high
 *   priority: high enough to still execute when the canary cannot, so it can
 *   capture a full diagnostic dump at the moment of the stall, then stop petting
 *   so the hardware watchdog recovers the board. It does NOT pet
 *   unconditionally - it pets only as a consequence of the low-priority
 *   heartbeat, so it does not mask starvation.
 *
 *   If even the supervisor is starved (an equal/higher-priority busy-loop, a
 *   long critical section, or IRQs disabled), the hardware watchdog still fires
 *   on its own - and the absence of a dump is itself the signal that the block
 *   was at the scheduler level rather than ordinary priority starvation.
 */

#ifndef WATCHDOG_SUPERVISOR_H
#define WATCHDOG_SUPERVISOR_H

#include <stdint.h>

/**
 * @brief Boot-time: inspect the reset cause and enforce the boot-loop limit.
 *
 * If this boot was caused by the watchdog, increments the persistent reset
 * counter (watchdog scratch[0]); if too many resets have happened without the
 * board ever reaching healthy uptime, disables the watchdog and traps in
 * CriticalErrorHandler. Otherwise (a clean power-on) clears the counter.
 *
 * Must be called once, early in OS_start(), before the scheduler starts.
 */
void WatchdogSupervisor_HandleBootResetCause(void);

/**
 * @brief Boot-time: arm the hardware watchdog.
 *
 * Call once from OS_start() after the supervisor task has been created, so the
 * first timeout window is pet as soon as the scheduler runs.
 */
void WatchdogSupervisor_Enable(void);

/**
 * @brief Task setup. Call once at the top of the supervisor task, before the
 *        periodic loop. Captures the task handle (for WatchdogSupervisor_
 *        RequestReset()), records the boot time and primes the first snapshot.
 */
void WatchdogSupervisor_Init(void);

/**
 * @brief One supervisor cycle. Call every supervisor period from the task loop.
 *
 * Honors an intentional reset request, tracks the heartbeat, decays the
 * boot-loop counter after healthy uptime, and either pets the watchdog or - on
 * a detected stall - records a breadcrumb, prints a live dump and stops petting.
 */
void WatchdogSupervisor_MainFunction(void);

/**
 * @brief Liveness heartbeat. The monitored low-priority task calls this once
 *        per fully completed loop iteration.
 */
void WatchdogSupervisor_ReportAlive(void);

/**
 * @brief Request an intentional, clean reset of the board.
 *
 * Clears the boot-loop counter (this reset is deliberate, not a fault) and
 * stops the watchdog being pet so the hardware reboots the board. Falls back to
 * a direct watchdog_reboot() if the watchdog is somehow not actually running.
 * The caller is expected to then block until the reset occurs.
 */
void WatchdogSupervisor_RequestReset(void);

#endif /* WATCHDOG_SUPERVISOR_H */
