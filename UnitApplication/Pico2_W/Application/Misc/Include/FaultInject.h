/**
 * File: FaultInject.h
 * Description: Compile-time-selected fault injection harness used to verify
 *              that FaultHandler.c captures and reports each kind of fault
 *              correctly. Default is FAULT_INJECT_NONE so production builds
 *              are unaffected.
 *
 *              To run an injection test, change the default of
 *              FAULT_INJECT_KIND below (or pass -DFAULT_INJECT_KIND=N at
 *              compile time), rebuild, flash, watch the UART for the live
 *              [FAULT] line, let the watchdog reboot, then confirm that the
 *              post-boot FaultHandler_ReportLastCrash() output matches what
 *              we asked for. Then put the macro back to FAULT_INJECT_NONE.
 */

#ifndef FAULT_INJECT_H
#define FAULT_INJECT_H

#define FAULT_INJECT_NONE             0
#define FAULT_INJECT_HARDFAULT_TASK   1  /* udf #0 from a task -> HardFault, PSP-stack path */
#define FAULT_INJECT_ASSERT           2  /* configASSERT(0) from a task                     */
#define FAULT_INJECT_STACK_OVF        3  /* bounded recursion past task stack base          */
#define FAULT_INJECT_MALLOC_FAIL      4  /* pvPortMalloc() with a request beyond the heap   */

#ifndef FAULT_INJECT_KIND
#define FAULT_INJECT_KIND FAULT_INJECT_NONE
#endif

/**
 * @brief Spawn the fault-injection task if a non-NONE kind is selected.
 *
 * The spawned task sleeps for ~2 s so the normal boot logs come out first,
 * then triggers the configured fault and never returns. When FAULT_INJECT_KIND
 * is FAULT_INJECT_NONE this is an empty function and the linker drops the
 * rest of the module.
 *
 * Call once from OS_start(), just before vTaskStartScheduler().
 */
void FaultInject_Start(void);

#endif /* FAULT_INJECT_H */
