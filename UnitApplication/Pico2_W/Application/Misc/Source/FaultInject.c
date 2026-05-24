/**
 * File: FaultInject.c
 * Description: Implementation of the fault-injection harness. See FaultInject.h
 *              for usage. Each branch in fault_inject_task() corresponds to one
 *              of the four fault paths FaultHandler.c is supposed to capture.
 */

#include "FaultInject.h"

#include "FreeRTOS.h"
#include "task.h"

#include "Common.h" /* LOG() */

#if (FAULT_INJECT_KIND != FAULT_INJECT_NONE)

/* The stack-overflow case wants a small stack so we can creep past the end
 * with a shallow, controlled recursion. Other kinds need enough stack for
 * LOG() and a function call or two. */
#if (FAULT_INJECT_KIND == FAULT_INJECT_STACK_OVF)
    #define FAULT_INJECT_STACK_WORDS    ((configSTACK_DEPTH_TYPE)256)  /* 1024 bytes */
#else
    #define FAULT_INJECT_STACK_WORDS    ((configSTACK_DEPTH_TYPE)512)  /* 2048 bytes */
#endif

#if (FAULT_INJECT_KIND == FAULT_INJECT_STACK_OVF)
/* Each call to recurse_overflow() puts a 32-byte volatile pad plus the
 * saved-register/return-address frame on the stack (~40-50 bytes total). With a
 * 1024-byte task stack, ~25 levels is enough to send SP just past the stack base.
 * The vTaskDelay() at the bottom forces a context switch while SP is still
 * below pxStack, which is when the kernel's overflow check fires. */
static __attribute__((noinline)) void recurse_overflow(uint32_t depth)
{
    volatile uint8_t frame_pad[32];
    frame_pad[0] = (uint8_t)depth;
    if (depth < 30)
    {
        recurse_overflow(depth + 1);
    }
    else
    {
        vTaskDelay(1);
    }
    (void)frame_pad[0];
}
#endif

static void fault_inject_task(void *unused)
{
    (void)unused;

    /* Wait so the normal boot logs (and FaultHandler_ReportLastCrash from the
     * previous boot, if any) come out before the injected fault muddies the
     * UART. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    LOG("[FaultInject] Triggering injection kind %d...\n", (int)FAULT_INJECT_KIND);

#if (FAULT_INJECT_KIND == FAULT_INJECT_HARDFAULT_TASK)
    /* udf #0 is the Armv8-M "Permanently Undefined" instruction. Guaranteed
     * UsageFault, which escalates to HardFault because UsageFault isn't
     * separately enabled in this project. */
    __asm volatile("udf #0" ::: "memory");
#elif (FAULT_INJECT_KIND == FAULT_INJECT_ASSERT)
    configASSERT(0);
#elif (FAULT_INJECT_KIND == FAULT_INJECT_STACK_OVF)
    recurse_overflow(0);
    /* If we somehow returned without the hook firing, force a context switch
     * so any pattern damage is detected. */
    taskYIELD();
#elif (FAULT_INJECT_KIND == FAULT_INJECT_MALLOC_FAIL)
    /* configTOTAL_HEAP_SIZE is 128 KB. 256 MB is comfortably beyond it, so
     * pvPortMalloc() must return NULL and call vApplicationMallocFailedHook(). */
    (void)pvPortMalloc((size_t)0x10000000);
#else
    #error "Unknown FAULT_INJECT_KIND"
#endif

    /* Unreachable - every branch above ends in a watchdog reboot. */
    for (;;)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void FaultInject_Start(void)
{
    BaseType_t rc = xTaskCreate(fault_inject_task,
                                "Inject",
                                FAULT_INJECT_STACK_WORDS,
                                NULL,
                                tskIDLE_PRIORITY + 1,
                                NULL);
    if (rc != pdPASS)
    {
        LOG("[FaultInject] xTaskCreate failed (%d)\n", (int)rc);
    }
}

#else  /* FAULT_INJECT_KIND == FAULT_INJECT_NONE */

void FaultInject_Start(void)
{
    /* No-op in production builds. */
}

#endif
