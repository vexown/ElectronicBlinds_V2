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
 */

#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include <stdint.h>

/* Print any crash info recorded by the previous boot (if any) and clear it.
 * Call once, early in main(), after stdio_init_all(). */
void FaultHandler_ReportLastCrash(void);

/* Hook entry points - record crash info, log, and reboot via watchdog. */
__attribute__((noreturn)) void FaultHandler_RecordAssert(const char *file, uint32_t line);
__attribute__((noreturn)) void FaultHandler_RecordStackOverflow(const char *task_name);
__attribute__((noreturn)) void FaultHandler_RecordMallocFailed(void);

#endif /* FAULT_HANDLER_H */
