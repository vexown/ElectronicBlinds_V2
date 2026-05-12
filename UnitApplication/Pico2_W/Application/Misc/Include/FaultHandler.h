/**
 * File: FaultHandler.h
 * Description: Captures crash context (HardFault registers, configASSERT, stack
 *              overflow, malloc failure) into the RP2350 watchdog scratch
 *              registers and reboots cleanly. After the reboot,
 *              FaultHandler_ReportLastCrash() prints what happened.
 *
 *              Scratch register layout used (scratch[0] reserved for the
 *              existing watchdog reset counter, scratch[4..7] are clobbered by
 *              watchdog_reboot()):
 *                  scratch[1] = magic | fault-type
 *                  scratch[2] = primary datum (PC / line / 0)
 *                  scratch[3] = secondary datum (LR / file-hash / first 4
 *                               chars of task name as ASCII)
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
