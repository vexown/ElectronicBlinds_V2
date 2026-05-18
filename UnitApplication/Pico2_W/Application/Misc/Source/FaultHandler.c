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
 * avoid pulling CMSIS into the rest of the codebase. 
 *
 * See RP2350 datasheet from details on the CFSR and HFSR Registers 
 * In short:
 *      CFSR = Configurable Fault Status Register - holds info on UsageFault, BusFault and MemManage exceptions
 *      HFSR = HardFault Status Register - shows the cause of any HardFaults 
 */
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

/**
 * @brief Hash a NUL-terminated string into a 32-bit value using FNV-1a.
 *        FNV-1a is a fast, non-cryptographic hash that turns an arbitrary-length 
 *        string into a fixed-size 32-bit fingerprint.
 *
 * Why we need a hash here:
 *   configASSERT() gives us a __FILE__ pointer like "Application/Foo/Bar.c"
 *   plus a line number. We only have one 32-bit scratch register to spare
 *   for the file identity, which is nowhere near enough to store the path
 *   itself. Instead we store a 32-bit fingerprint of the path and, on the
 *   next boot, the report code prints that fingerprint. Match it against
 *   `grep -rn` over the source tree (or a precomputed map) to recover the
 *   original filename.
 *
 * Why FNV-1a specifically:
 *   - Tiny: a couple of arithmetic ops per byte, no tables, no allocations,
 *     safe to call from awkward contexts.
 *   - Good enough distribution for short ASCII paths so different files
 *     almost never collide to the same 32-bit value in practice.
 *   - Public-domain, well-known constants - the two magic numbers below are
 *     not arbitrary, they are the standardised FNV-1a parameters.
 *
 * The two magic numbers (from the FNV-1a specification - RFC 9923):
 *   - 0x811C9DC5 = FNV offset basis: the seed value the hash starts from.
 *   - 0x01000193 = FNV prime:        the multiplier applied each iteration.
 * These were chosen by the FNV authors to give good avalanche behaviour on
 * 32-bit outputs; do not change them or the hash stops being FNV-1a.
 *
 * @param s   NUL-terminated input string; NULL is tolerated and yields 0.
 * @return    32-bit hash, or 0 if @p s is NULL.
 */
static uint32_t fnv1a32(const char *s)
{
    /* Start from the FNV-1a offset basis. */
    uint32_t h = 0x811C9DC5U;

    /* Defensive: callers may hand us a NULL __FILE__ in odd build configs. */
    if (s == NULL) return 0;

    /* FNV-1a core loop: XOR-then-multiply for every byte of the string.
     * The "1a" variant XORs first and multiplies after (FNV-1 does the
     * reverse); 1a has measurably better avalanche, which is why it is
     * the recommended variant. 
     */
    while (*s) // will naturally end at the NULL terminator of the hash input string
    {
        /* Explanation of (uint32_t)(uint8_t)(*s) conversion below:
        *
        *  INTEGER CONVERSION & VALUE PRESERVATION BEHAVIOR (C11 §6.3.1.3):
        *
        * 1. Direct Unsigned Promotion (Sign Extension):
        *    When converting a negative signed type (e.g., 'char' at 0x80 = -128) 
        *    directly to a larger unsigned type like 'uint32_t', C11 §6.3.1.3 p2 
        *    mandates modular wrapping: Value + (MAX + 1).
        *    Calculation: -128 + 4294967296 = 4294967168 (Hex: 0xFFFFFF80).
        *    This effectively triggers hardware sign extension, altering the hex pattern.
        *
        * 2. Same-Width Cast (Modulo Alignment):
        *    Converting 'char' (-128) to 'uint8_t' applies the same rule:
        *    Calculation: -128 + 256 = 128 (Hex: 0x80).
        *    The underlying bit pattern remains 0x80, but the value becomes positive.
        *
        * 3. Preserving Raw Hex/Byte Value (Safe Widening):
        *    To widen a signed byte while preserving its literal hex value (0x80), 
        *    you must cast to 'uint8_t' first, then to 'uint32_t'. 
        *    The intermediate 'uint8_t' holds a value of 128. Moving from 'uint8_t' 
        *    to 'uint32_t' invokes C11 §6.3.1.3 p1 (Value Preservation), because 
        *    128 fits perfectly in the target type. Result: 128 (Hex: 0x00000080).
        */
        h ^= (uint32_t)(uint8_t)(*s++);  /* fold next byte into the state */
        h *= 0x01000193U;                /* mix bits via the FNV prime    */
    }

    return h;
}

/**
 * @brief Pack the first up-to-4 characters of a string into a single uint32_t.
 *
 * Layout: LSB = first char, MSB = fourth char. For example "Moni" packs as
 *   's[0]'='M' (0x4D)  -> bits  7..0
 *   's[1]'='o' (0x6F)  -> bits 15..8
 *   's[2]'='n' (0x6E)  -> bits 23..16
 *   's[3]'='i' (0x69)  -> bits 31..24
 * yielding 0x696E6F4D.
 *
 * Why we do this:
 *   On a FreeRTOS stack-overflow hook we receive the task name as a pointer
 *   but we only have one 32-bit scratch slot to remember it across the
 *   reboot. Storing the first four characters is enough to identify the
 *   offending task in practice (FreeRTOS task names are typically short and
 *   start uniquely - "Moni", "Wifi", "Stat", etc.). Unpacking on the next
 *   boot reverses the layout to print a recognisable name.
 *
 * @param s   NUL-terminated input string; NULL is tolerated and yields 0.
 * @return    The packed 32-bit value as described above.
 */
static uint32_t pack_first4(const char *s)
{
    uint32_t out = 0;

    if (s == NULL) return 0;

    for (int i = 0; ((i < 4) && (s[i] != '\0')); i++)
    {
        out |= ((uint32_t)(uint8_t)s[i]) << (i * 8);
    }
    
    return out;
}

/**
 * @brief Trigger a clean watchdog-driven reboot and never return.
 *
 * Called at the tail of every fault path (HardFault, assert, stack
 * overflow, malloc failure). watchdog_reboot(pc=0, sp=0, delay_ms=10) asks
 * the SDK for a regular reset after 10 ms; that small delay gives the UART
 * transmit FIFO time to drain so the live "[FAULT] ..." line we just
 * printed actually reaches the terminal before the reset cuts the link.
 *
 * Important: watchdog_reboot() only writes scratch[4..7] (sentinel + PC/SP
 * handoff consumed by the boot ROM), so our crash record in scratch[1..3]
 * is preserved across the reset and can be reported on the next boot.
 *
 * The infinite loop after the call is unreachable but required so the
 * compiler is happy with the noreturn contract until the watchdog fires.
 */
__attribute__((noreturn)) static void reboot_now(void)
{
    watchdog_reboot(0, 0, 10);
    for ( ;; )
    {
        tight_loop_contents();
    }
}

/**
 * @brief Write a NUL-terminated string straight to the default UART.
 *
 * This is a deliberate alternative to printf()/puts() for use from a
 * HardFault context. Inside an exception we may have arrived with corrupt
 * state, interrupts in an awkward configuration, or with the stdio
 * subsystem's internal mutex held by a preempted task - calling printf()
 * there can deadlock or crash again. uart_putc_raw() is a thin register
 * write to the UART data register with no locking, no formatting, and no
 * dependency on FreeRTOS, so it is safe in nearly any context.
 *
 * Use this only on the fault path. Outside of fault context the normal
 * printf() path is fine and gives nicer output.
 *
 * @param s   NUL-terminated string to emit; must not be NULL.
 */
static void emit_raw(const char *s)
{
    while (*s)
    {
        uart_putc_raw(uart_default, *s++);
    }
}

/**
 * @brief Emit a 32-bit value to the UART as "0x" + 8 lowercase hex digits.
 *
 * Companion to emit_raw() for the fault-path logger. printf("%08lx", ...)
 * would be simpler but pulls in the full formatted-output path (locks,
 * heap-backed buffers, varargs machinery) which is unsafe from a
 * HardFault. Doing the hex conversion by hand keeps the operation to a
 * fixed-size stack buffer and a sequence of UART register writes - no
 * allocations, no locks, no recursion.
 *
 * Output is always exactly 10 characters ("0xDEADBEEF"-style), no newline,
 * no leading or trailing space; callers concatenate spacing/labels via
 * emit_raw().
 *
 * @param v   The 32-bit value to print.
 */
static void emit_hex32(uint32_t v)
{
    static const char digits[] = "0123456789abcdef";
    char buf[10];

    /* Compose the hex string */
    buf[0] = '0';
    buf[1] = 'x';
    for (int i = 0; i < 8; i++)
    {
        buf[2 + i] = digits[(v >> (28 - (i * 4))) & 0xFU];
    }

    /* Send the hex string */
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
