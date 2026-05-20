/**
 * File: FaultHandler.h
 * Description: Captures crash context (HardFault registers, configASSERT, stack
 *              overflow, malloc failure) into the RP2350 watchdog scratch
 *              registers and reboots cleanly. After the reboot,
 *              FaultHandler_ReportLastCrash() prints what happened.
 * 
 *              The watchdog peripheral contains eight 32-bit general-purpose 
 *              scratch registers, SCRATCH0 through SCRATCH7, at offsets 0x0c 
 *              through 0x28 in the watchdog register block. Each one is plain RW 
 *              with a reset value of 0x00000000.
 * 
 *              The most important property of these scratch registers is that they retain
 *              their values across certain types of reset, including the software-triggered reset we use here.
 *              
 *              To be precise, based on the RP2350 datasheet, the value of watchdog scratch registers is...
 * 
 *              Preserved across:
 *
 *                  - Watchdog-triggered system reset (via PSM_WDSEL) — the typical case, including the SDK default and watchdog_reboot()
 *                  - Watchdog-triggered subsystem reset (via RESETS_WDSEL)
 *                  - Any other system or subsystem reset not caused by the watchdog
 * 
 *              Cleared by:
 * 
 *                  - Watchdog-triggered chip-level reset (via POWMAN_WATCHDOG)
 *                  - Any other chip-level reset (POR/BOR, glitch detector, debugger, etc.)
 *                  - rst_n_run event — toggling the RUN pin or cycling DVDD
 *
 *              Scratch register layout used:
 *
 *                  - scratch[0]    - reserved (existing watchdog reset counter)
 *                  - scratch[1]    = magic | fault-type
 *                  - scratch[2]    = primary datum (PC / line / 0)
 *                  - scratch[3]    = secondary datum (LR / file-hash / first 4 chars of task name as ASCII)
 *                  - scratch[4..7] - owned by the SDK/boot ROM - not for user data
 *                                    watchdog_reboot() writes a sentinel + PC/SP handoff here that
 *                                    the boot ROM consumes on the next reset, so
 *                                    these slots cannot be used for crash data.
 *
 *              ----------------------------------------------------------------
 *              Fault types handled
 *              ----------------------------------------------------------------
 *
 *              This module captures four kinds of fault. They split into two
 *              categories by *who* detects them:
 *
 *                A) Hardware-detected  -> the Cortex-M33 CPU itself raises an
 *                                         exception.
 *                B) Software-detected  -> FreeRTOS notices a problem and calls
 *                                         a hook function. The CPU is still
 *                                         running fine; the RTOS is reporting a
 *                                         programming/resource error.
 *
 *              (A) HardFault
 *              -------------
 *              A HardFault is a CPU exception - the processor hit something it
 *              physically cannot execute correctly and vectored to the
 *              HardFault handler. Typical causes:
 *
 *                  - Dereferencing a bad pointer (NULL, dangling, unaligned).
 *                  - Calling through a corrupted function pointer, or returning
 *                    to a corrupted address (jump to non-executable memory).
 *                  - Executing an undefined / illegal instruction.
 *                  - A bus error on an instruction fetch or data access.
 *                  - Divide-by-zero or unaligned access, when trapping of those
 *                    is enabled.
 *
 *              The Cortex-M33 also has finer-grained configurable fault
 *              exceptions (MemManage, BusFault, UsageFault, SecureFault). When
 *              those are not separately enabled - or a fault occurs while
 *              already handling a fault - they "escalate" to HardFault. Since
 *              this project installs only a HardFault handler, HardFault is the
 *              single catch-all for every CPU fault. It captures the stacked
 *              PC (faulting instruction) and LR, plus the CFSR/HFSR status
 *              registers, to pinpoint the cause.
 *
 *              (B) configASSERT failure
 *              ------------------------
 *              configASSERT() is NOT a CPU exception - it is a FreeRTOS
 *              assertion macro, the RTOS equivalent of the C standard assert().
 *              FreeRTOS sprinkles configASSERT() checks throughout the kernel
 *              to verify invariants and catch API misuse: e.g. a NULL queue
 *              handle, an interrupt priority outside the syscall-safe range, or
 *              calling a FromISR / non-FromISR function in the wrong context.
 *              You can use it in application code too. When the asserted
 *              condition is false, configASSERT() routes (via vAssertCalled())
 *              to FaultHandler_RecordAssert(). It is a deliberate, software
 *              tripwire that catches a programming error early - before it can
 *              snowball into silent corruption or a HardFault.
 *
 *              (C) Stack overflow
 *              ------------------
 *              Each FreeRTOS task has its own fixed-size stack. If a task uses
 *              more stack than was allocated for it, it overflows into
 *              neighbouring memory. With configCHECK_FOR_STACK_OVERFLOW
 *              enabled, FreeRTOS checks for this at every context switch -
 *              either by bounds-checking the task's stack pointer, or by
 *              verifying that a known fill pattern at the end of the stack is
 *              still intact. On detection it calls
 *              vApplicationStackOverflowHook(), which routes to
 *              FaultHandler_RecordStackOverflow(). Note this is best-effort:
 *              the overflow has already happened by the time it is noticed.
 *
 *              (D) malloc failure (heap exhaustion)
 *              ------------------------------------
 *              FreeRTOS allocates task stacks, queues, timers, etc. from its
 *              own heap via pvPortMalloc(). When a request cannot be satisfied
 *              because the heap is full, pvPortMalloc() returns NULL and, with
 *              configUSE_MALLOC_FAILED_HOOK enabled, calls
 *              vApplicationMallocFailedHook(), which routes to
 *              FaultHandler_RecordMallocFailed(). This is a resource-exhaustion
 *              report, not a crash - but it is captured so an out-of-memory
 *              condition is never lost silently.
 */

#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <stdint.h>

/**
 * @brief Print any crash info recorded by the previous boot and clear it.
 *
 * Inspects watchdog scratch[1..3] for a valid fault record (identified by the
 * 0xFA017XXX magic in scratch[1]). If one is found, the human-readable
 * description is printed to stdout - including PC/LR for HardFaults, file
 * hash + line number for asserts, a packed task name for stack overflows, or
 * a notice for malloc failures. The scratch slots are then zeroed so the
 * same crash is not re-reported on subsequent reboots.
 *
 * If no valid record is present (e.g. fresh power-on, or this boot was not
 * preceded by a fault-driven reboot), the function returns silently.
 *
 * Must be called exactly once, early in main(), after stdio_init_all() so
 * that the print actually reaches the user's terminal.
 */
void FaultHandler_ReportLastCrash(void);

/**
 * @brief Record a configASSERT() failure and reboot via the watchdog.
 *
 * Called from vAssertCalled(). Stores:
 *   - scratch[1] = magic | FAULT_TYPE_ASSERT
 *   - scratch[2] = ulLine (asserting line number)
 *   - scratch[3] = FNV-1a 32-bit hash of pcFile (the file path)
 *
 * The file path itself cannot fit in 32 bits, so a fingerprint is stored
 * instead; on the next boot the hash is printed and the user can
 * `grep -rn` the source tree (or consult a precomputed map) to recover the
 * original filename. A best-effort line is also printed live, then the
 * function triggers a watchdog reboot and never returns.
 *
 * @param file   __FILE__ value passed in by configASSERT(); NULL is tolerated.
 * @param line   __LINE__ value passed in by configASSERT().
 */
__attribute__((noreturn)) void FaultHandler_RecordAssert(const char *file, uint32_t line);

/**
 * @brief Record a FreeRTOS stack-overflow event and reboot via the watchdog.
 *
 * Called from vApplicationStackOverflowHook(). Stores:
 *   - scratch[1] = magic | FAULT_TYPE_STACK_OVF
 *   - scratch[2] = 0 (unused)
 *   - scratch[3] = first 4 chars of task_name packed as little-endian ASCII
 *
 * The task name is truncated to 4 characters to fit a single scratch slot;
 * in practice FreeRTOS task names start uniquely enough that 4 chars
 * identify the offender (e.g. "Moni", "Wifi"). A live message is also
 * printed before the watchdog reboot, after which this function does not
 * return.
 *
 * @param task_name  Task name pointer supplied by the FreeRTOS hook; NULL
 *                   is tolerated.
 */
__attribute__((noreturn)) void FaultHandler_RecordStackOverflow(const char *task_name);

/**
 * @brief Record an out-of-heap (pvPortMalloc) failure and reboot via the watchdog.
 *
 * Called from vApplicationMallocFailedHook(). Stores:
 *   - scratch[1] = magic | FAULT_TYPE_MALLOC_FAIL
 *   - scratch[2] = 0
 *   - scratch[3] = 0
 *
 * There is no useful payload to capture - the failure simply means the
 * FreeRTOS heap is exhausted. The next boot will print a notice so the
 * event is not silently lost. Triggers a watchdog reboot and never returns.
 */
__attribute__((noreturn)) void FaultHandler_RecordMallocFailed(void);

#endif /* FAULT_HANDLER_H */
