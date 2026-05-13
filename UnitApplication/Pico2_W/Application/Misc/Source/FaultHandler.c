/**
 * File: FaultHandler.c
 * Description: See FaultHandler.h.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

#include "FaultHandler.h"

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* Upper 20 bits = magic ("FA017"), lower 12 bits = fault type. */
/* The magic value helps distinguish a real fault record from random/zero data in the
 * scratch registers after a power cycle, brown-out, glitch or other random crash. */
#define FAULT_MAGIC_BASE        0xFA017000U
#define FAULT_MAGIC_MASK        0xFFFFF000U
#define FAULT_TYPE_MASK         0x00000FFFU
#define FAULT_TAG(type)         (FAULT_MAGIC_BASE | (uint32_t)(type))

#define FAULT_TYPE_HARDFAULT    0x001U
#define FAULT_TYPE_ASSERT       0x002U
#define FAULT_TYPE_STACK_OVF    0x003U
#define FAULT_TYPE_MALLOC_FAIL  0x004U

/* Standard Cortex-M System Control Block registers. Accessed directly to
 * avoid pulling CMSIS into the rest of the codebase. */
#define SCB_CFSR_ADDR           0xE000ED28U
#define SCB_HFSR_ADDR           0xE000ED2CU

/*******************************************************************************/
/*                       STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/

static uint32_t fnv1a32(const char *s);
static uint32_t pack_first4(const char *s);
__attribute__((noreturn)) static void reboot_now(void);
__attribute__((noreturn, used)) void FaultHandler_HardFaultC(uint32_t *frame);

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

static uint32_t fnv1a32(const char *s)
{
    uint32_t h = 0x811C9DC5U;
    if (s == NULL)
    {
        return 0;
    }
    while (*s)
    {
        h ^= (uint32_t)(uint8_t)(*s++);
        h *= 0x01000193U;
    }
    return h;
}

/* Pack the first up-to-4 characters of s into a u32 (LSB = first char), so
 * "Moni" -> 0x696E6F4D. Lets the post-boot log show recognisable task names. */
static uint32_t pack_first4(const char *s)
{
    uint32_t out = 0;
    if (s == NULL)
    {
        return 0;
    }
    for (int i = 0; i < 4 && s[i] != '\0'; i++)
    {
        out |= ((uint32_t)(uint8_t)s[i]) << (i * 8);
    }
    return out;
}

__attribute__((noreturn)) static void reboot_now(void)
{
    /* 10 ms gives UART tx FIFO time to drain. watchdog_reboot writes
     * scratch[4..7] only - our scratch[1..3] crash info survives. */
    watchdog_reboot(0, 0, 10);
    for ( ;; )
    {
        tight_loop_contents();
    }
}

/* Minimal raw UART emit. Used from HardFault context where the stdio mutex
 * may not be safe to acquire. */
static void emit_raw(const char *s)
{
    while (*s)
    {
        uart_putc_raw(uart_default, *s++);
    }
}

static void emit_hex32(uint32_t v)
{
    static const char digits[] = "0123456789abcdef";
    char buf[10];
    buf[0] = '0';
    buf[1] = 'x';
    for (int i = 0; i < 8; i++)
    {
        buf[2 + i] = digits[(v >> (28 - (i * 4))) & 0xFU];
    }
    for (int i = 0; i < 10; i++)
    {
        uart_putc_raw(uart_default, buf[i]);
    }
}

/* C side of the HardFault handler. frame points at the stacked exception
 * frame: { R0, R1, R2, R3, R12, LR, PC, xPSR, ... }. */
__attribute__((noreturn, used)) void FaultHandler_HardFaultC(uint32_t *frame)
{
    const uint32_t pc   = frame[6];
    const uint32_t lr   = frame[5];
    const uint32_t cfsr = *(volatile uint32_t *)SCB_CFSR_ADDR;
    const uint32_t hfsr = *(volatile uint32_t *)SCB_HFSR_ADDR;

    watchdog_hw->scratch[1] = FAULT_TAG(FAULT_TYPE_HARDFAULT);
    watchdog_hw->scratch[2] = pc;
    watchdog_hw->scratch[3] = lr;

    emit_raw("\n[FAULT] HardFault PC=");
    emit_hex32(pc);
    emit_raw(" LR=");
    emit_hex32(lr);
    emit_raw(" CFSR=");
    emit_hex32(cfsr);
    emit_raw(" HFSR=");
    emit_hex32(hfsr);
    emit_raw("\n");

    reboot_now();
}

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

void FaultHandler_ReportLastCrash(void)
{
    const uint32_t tag   = watchdog_hw->scratch[1];
    const uint32_t data1 = watchdog_hw->scratch[2];
    const uint32_t data2 = watchdog_hw->scratch[3];

    if ((tag & FAULT_MAGIC_MASK) != FAULT_MAGIC_BASE)
    {
        return;
    }

    const uint32_t type = tag & FAULT_TYPE_MASK;
    printf("\n---- Previous boot crashed ----\n");
    switch (type)
    {
        case FAULT_TYPE_HARDFAULT:
            printf("Type: HardFault\n");
            printf("PC=0x%08lx  LR=0x%08lx\n",
                   (unsigned long)data1, (unsigned long)data2);
            printf("Resolve PC with: arm-none-eabi-addr2line -e <elf> 0x%08lx\n",
                   (unsigned long)data1);
            break;
        case FAULT_TYPE_ASSERT:
            printf("Type: configASSERT\n");
            printf("Line %lu (file hash 0x%08lx)\n",
                   (unsigned long)data1, (unsigned long)data2);
            break;
        case FAULT_TYPE_STACK_OVF:
        {
            /* data2 holds first 4 chars of task name as packed ASCII. */
            char name[5] = {0};
            for (int i = 0; i < 4; i++)
            {
                name[i] = (char)((data2 >> (i * 8)) & 0xFFU);
            }
            printf("Type: Stack overflow\n");
            printf("Task name starts with: '%s'\n", name);
            break;
        }
        case FAULT_TYPE_MALLOC_FAIL:
            printf("Type: pvPortMalloc failed (out of FreeRTOS heap)\n");
            break;
        default:
            printf("Type: unknown (0x%lx)\n", (unsigned long)type);
            break;
    }
    printf("-------------------------------\n\n");

    /* Clear so we don't re-report on subsequent reboots. */
    watchdog_hw->scratch[1] = 0;
    watchdog_hw->scratch[2] = 0;
    watchdog_hw->scratch[3] = 0;
}

__attribute__((noreturn)) void FaultHandler_RecordAssert(const char *file, uint32_t line)
{
    watchdog_hw->scratch[1] = FAULT_TAG(FAULT_TYPE_ASSERT);
    watchdog_hw->scratch[2] = line;
    watchdog_hw->scratch[3] = fnv1a32(file);

    printf("[FAULT] configASSERT failed at %s:%lu\n",
           (file != NULL) ? file : "?", (unsigned long)line);

    reboot_now();
}

__attribute__((noreturn)) void FaultHandler_RecordStackOverflow(const char *task_name)
{
    watchdog_hw->scratch[1] = FAULT_TAG(FAULT_TYPE_STACK_OVF);
    watchdog_hw->scratch[2] = 0;
    watchdog_hw->scratch[3] = pack_first4(task_name);

    printf("[FAULT] Stack overflow in task '%s'\n",
           (task_name != NULL) ? task_name : "?");

    reboot_now();
}

__attribute__((noreturn)) void FaultHandler_RecordMallocFailed(void)
{
    watchdog_hw->scratch[1] = FAULT_TAG(FAULT_TYPE_MALLOC_FAIL);
    watchdog_hw->scratch[2] = 0;
    watchdog_hw->scratch[3] = 0;

    printf("[FAULT] pvPortMalloc returned NULL (FreeRTOS heap exhausted)\n");

    reboot_now();
}

/* Cortex-M HardFault entry. Picks MSP or PSP based on EXC_RETURN bit 2, then
 * tail-calls the C handler with the stacked frame pointer in r0.
 * Overrides the weak symbol from pico-sdk's crt0.S. */
__attribute__((naked)) void isr_hardfault(void)
{
    __asm volatile (
        "tst   lr, #4                       \n"
        "ite   eq                           \n"
        "mrseq r0, msp                      \n"
        "mrsne r0, psp                      \n"
        "b     FaultHandler_HardFaultC      \n"
    );
}
