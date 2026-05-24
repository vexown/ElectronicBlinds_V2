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

/* Needed by the HardFault path for the Armv8-M-Mainline UFSR.STKOF case:
 * we read pxCurrentTCB (with a NULL guard) and call pcTaskGetName() to label
 * the offending task. Everything else in this file is FreeRTOS-free; these
 * two includes only get used inside the STKOF branch. */
#include "FreeRTOS.h"
#include "task.h"

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

/* CFSR bit 20 = UFSR.STKOF (stack-overflow UsageFault, Armv8-M Mainline only).
 * Set by the CPU when an instruction would push SP below MSPLIM/PSPLIM. */
#define CFSR_UFSR_STKOF         (1U << 20)

/* Stack-overflow source discriminator stored in scratch[2] for FAULT_TYPE_STACK_OVF.
 * 0 = FreeRTOS pattern-check at context switch; 1 = Cortex-M33 hardware STKOF. */
#define STK_OVF_SOURCE_FREERTOS 0U
#define STK_OVF_SOURCE_STKOF    1U

/* Kernel-internal pointer to the currently-running task's TCB. We only
 * dereference it via pcTaskGetName() and we guard against NULL (the scheduler
 * may not be running yet when a fault fires), so reading it from a fault
 * context is safe. Declared here to avoid pulling TCB_t into this file. */
extern TaskHandle_t volatile pxCurrentTCB;

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
        tight_loop_contents(); // pico-sdk no-op function
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

/**
 * @brief C-language half of the HardFault handler.
 *
 * Receives a pointer to the exception stack frame that the CPU automatically
 * pushed on entry into HardFault. This long comment documents both what this
 * function does and the surrounding Cortex-M exception model that makes it
 * possible to write a fault handler in plain C.
 *
 * --------------------------------------------------------------------------
 *  1. Context stacking (the "exception stack frame")
 * --------------------------------------------------------------------------
 *
 * References:
 *   - Armv8-M Architecture Reference Manual (DDI0553), section
 *     "B3.18  Exception handling" - the formal definition.
 *   - Arm Cortex-M33 Devices Generic User Guide (DUI0913),
 *     "Exception entry" - friendlier version.
 *   https://developer.arm.com/documentation/100235/0100/The-Cortex-M33-Processor/Exception-model/Exception-entry-and-return/Exception-entry-?lang=en 
 *
 * From the Cortex-M33 GUG:
 *   "When the processor takes an exception, unless the exception is a
 *    tail-chained or a late-arriving exception, the processor pushes
 *    information onto the current stack. This operation is referred to as
 *    stacking and the structure of the data stacked is referred as the
 *    stack frame. In parallel to the stacking operation, the processor
 *    performs a vector fetch that reads the exception handler start address
 *    from the vector table. When stacking is complete, the processor starts
 *    executing the exception handler."
 *
 * The "current stack" is whichever of the two stacks was active at the
 * moment of the fault: the Main Stack Pointer (MSP, used by ISRs and pre-RTOS
 * startup) or the Process Stack Pointer (PSP, used by FreeRTOS tasks). The
 * naked assembly trampoline isr_hardfault() picks the right one by testing
 * bit 2 (SPSEL) of the EXC_RETURN value left in LR by the CPU, then tail-
 * calls here with that pointer in r0.
 *
 * Basic ("integer-only") frame layout, 8 words = 32 bytes:
 *
 *      offset  index   contents
 *      0x00    [0]     R0
 *      0x04    [1]     R1
 *      0x08    [2]     R2
 *      0x0C    [3]     R3
 *      0x10    [4]     R12
 *      0x14    [5]     LR  - the interrupted code's real link register
 *      0x18    [6]     PC  - return address (see section 3 below)
 *      0x1C    [7]     xPSR (RETPSR) - see section 4 below
 *
 * If the FPU was active at fault time, the CPU pushes an "extended" frame
 * that also includes S0..S15 + FPSCR (and possibly more under Armv8-M with
 * the Security Extension). We don't use the FPU on this project, so the
 * basic frame is what we get.
 *
 * --------------------------------------------------------------------------
 *  2. Why exactly these registers? -> AAPCS, and "C handlers for free"
 * --------------------------------------------------------------------------
 *
 * The pushed set - R0..R3, R12, LR, PC, xPSR - is exactly the caller-saved
 * set under the Arm Procedure Call Standard (AAPCS). Caller-saved means
 * "any normal C function is allowed to clobber these without saving them".
 * The callee-saved registers (R4..R11) are NOT in the frame because any C
 * function that wants to use them is already obliged by AAPCS to save them
 * in its own prologue.
 *
 * Combine those two halves and you get a remarkable property: a Cortex-M
 * exception handler can be written as a plain C function. The CPU saves
 * the caller-saved half; the compiler-emitted prologue saves whichever
 * callee-saved registers the handler actually touches. Together, the full
 * register context of the interrupted code is preserved across the handler
 * with no special attribute, no __attribute__((interrupt)), no hand-written
 * prologue. This is why every entry in the pico-sdk vector table is just a
 * plain `void name(void)` function pointer.
 *
 * (The only reason isr_hardfault() in this file IS hand-written assembly
 * is that we need to inspect LR/EXC_RETURN _before_ the compiler-emitted
 * prologue overwrites it - to pick MSP vs PSP. Plain C handlers that don't
 * need to know which stack they were called from need no assembly at all.)
 *
 * --------------------------------------------------------------------------
 *  3. The "PC" slot (frame[6]) and the LR slot (frame[5])
 * --------------------------------------------------------------------------
 *
 * Two related-but-different things both get called "PC", which is a common
 * source of confusion:
 *
 *   a) The live PC register (R15) - the program counter you learned about.
 *      During the handler it points to handler code, as you would expect.
 *      Nothing about PC's normal semantics changes.
 *
 *   b) The PC slot in the exception frame (frame[6]) - a saved 32-bit word
 *      the CPU wrote at exception entry. The Armv8-M ARM calls this slot
 *      "ReturnAddress"; the Cortex-M33 GUG calls it "PC" because it is the
 *      value the CPU will reload into the live PC on exception return.
 *      The architecture (B3.18) defines what value the CPU stores there
 *      depending on the kind of exception:
 *
 *          - Asynchronous interrupt (SysTick, external IRQ, PendSV):
 *            address of the NEXT instruction to execute.
 *          - Synchronous precise fault (HardFault, BusFault, UsageFault,
 *            MemManage on instruction fetch, undefined opcode, etc.):
 *            address of the FAULTING instruction itself.
 *          - SVC / BKPT: address of the instruction AFTER the SVC/BKPT.
 *
 *      HardFault is a synchronous precise fault, so frame[6] for us is the
 *      faulting PC. That is what makes it useful for triage - run
 *      `arm-none-eabi-addr2line -e <elf> <PC>` against the printed value
 *      and you get the exact source line that blew up.
 *
 * Similarly there are two LR-shaped values:
 *
 *   a) The live LR register on entry to the handler holds the EXC_RETURN
 *      payload (e.g. 0xFFFFFFFD on Armv8-M Mainline), not a normal return
 *      address. Its bits encode SPSEL (which stack), Mode (thread/handler),
 *      FType (FP frame or not), and on Armv8-M S/ES (security domain).
 *      `bx lr` with EXC_RETURN is how the handler triggers exception
 *      return. The trampoline tests bit 2 (SPSEL) before any C code runs.
 *
 *   b) The LR slot in the frame (frame[5]) holds the interrupted code's
 *      REAL R14 - i.e. the genuine link-register value the faulting
 *      function was using to know where to return to its caller. We print
 *      this as "LR=" to help walk one frame up the call stack during
 *      post-mortem analysis.
 *
 * --------------------------------------------------------------------------
 *  4. xPSR / RETPSR (frame[7])
 * --------------------------------------------------------------------------
 *
 * Same value-from-two-views story: the GUG calls this slot "xPSR" because
 * that is where the value will be restored on return; the Armv8-M ARM
 * calls it "RETPSR" because the stacked copy has one extra bit defined.
 *
 * Specifically, AAPCS requires the SP to be 8-byte aligned at any public
 * function boundary, including exception handlers. On exception entry the
 * CPU forces 8-byte alignment by adding 4 bytes of padding before stacking
 * if the SP was only 4-byte aligned, and it records whether that padding
 * was applied in bit 9 of the stacked RETPSR. The live xPSR register does
 * not have a meaningful bit 9; only the stacked copy does. On exception
 * return the CPU consults bit 9 of frame[7] to undo the padding.
 *
 * We do not read frame[7] in this handler. It is documented here for the
 * benefit of anyone extending the handler later who needs to recover the
 * exact pre-fault SP, or to print the flags (N/Z/C/V/Q) or IT state of the
 * faulting code.
 *
 * --------------------------------------------------------------------------
 *  5. Where the fault came from -> CFSR and HFSR
 * --------------------------------------------------------------------------
 *
 * The stack frame tells us WHERE the fault happened (the faulting PC, the
 * caller via LR). It does NOT tell us WHY. For that the Cortex-M System
 * Control Block exposes two fixed-address status registers:
 *
 *   - CFSR @ 0xE000ED28 - Configurable Fault Status Register.
 *     Three sub-bytes for MemManage, BusFault and UsageFault flags;
 *     bits like IBUSERR (instruction-bus error), PRECISERR (precise data
 *     access error), INVSTATE (invalid Thumb state), DIVBYZERO, UNALIGNED.
 *   - HFSR @ 0xE000ED2C - HardFault Status Register.
 *     Bit FORCED tells you the original fault was escalated to a HardFault
 *     (e.g. UsageFault arrived but UsageFault is disabled, or a fault
 *     happened inside an already-running fault handler). Bit VECTTBL means
 *     a fault occurred during the initial vector fetch.
 *
 * Snapshotting both registers before we reboot is what gives a
 * post-mortem analysis a real chance of explaining the root cause.
 *
 * --------------------------------------------------------------------------
 *  6. Actions taken by this function (in order)
 * --------------------------------------------------------------------------
 *
 *   1. Read PC and LR out of the stacked frame (frame[6], frame[5]).
 *   2. Snapshot CFSR and HFSR from the SCB.
 *   3. Write magic|FAULT_TYPE_HARDFAULT, PC, and LR into watchdog scratch
 *      [1..3] so the next boot can report them via
 *      FaultHandler_ReportLastCrash().
 *   4. Emit a live "[FAULT] HardFault PC=... LR=... CFSR=... HFSR=..."
 *      line via the raw UART path (safe from exception context).
 *   5. Reboot through the watchdog.
 *
 * @param frame   Pointer to the stacked exception frame (MSP or PSP),
 *                supplied by the isr_hardfault() trampoline in r0.
 */
__attribute__((noreturn, used)) void FaultHandler_HardFaultC(uint32_t *frame)
{
    const uint32_t pc   = frame[6];
    const uint32_t lr   = frame[5];
    const uint32_t cfsr = *(volatile uint32_t *)SCB_CFSR_ADDR;
    const uint32_t hfsr = *(volatile uint32_t *)SCB_HFSR_ADDR;

    /* Armv8-M Mainline hardware stack-overflow trap. The Cortex-M33 has
     * MSPLIM/PSPLIM stack-limit registers; the FreeRTOS port sets PSPLIM to
     * each task's stack base on context switch, so when a recursion or large
     * local would push SP below the base, the CPU raises a UsageFault with
     * CFSR.UFSR.STKOF set. This fires synchronously, in nanoseconds - long
     * before FreeRTOS's configCHECK_FOR_STACK_OVERFLOW=2 pattern check would
     * notice at the next context switch.
     *
     * In this case the PC/LR words the CPU tried to stack are unreliable
     * (the stacking itself failed below the limit), so falling through to
     * the normal HardFault path would print a misleading "PC=0xa5a5a5a5".
     * Instead we re-route to the stack-overflow record format, label the
     * source as STKOF, and capture the running task's name so the report
     * stays accurate. */
    if (cfsr & CFSR_UFSR_STKOF)
    {
        const char *name = (pxCurrentTCB != NULL) ? pcTaskGetName(NULL) : NULL;

        watchdog_hw->scratch[1] = FAULT_TAG(FAULT_TYPE_STACK_OVF);
        watchdog_hw->scratch[2] = STK_OVF_SOURCE_STKOF;
        watchdog_hw->scratch[3] = pack_first4(name);

        emit_raw("\n[FAULT] Stack overflow (hardware STKOF) in task '");
        emit_raw((name != NULL) ? name : "?");
        emit_raw("'\n");

        reboot_now();
    }

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

    if ((tag & FAULT_MAGIC_MASK) != FAULT_MAGIC_BASE) return;

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
            /* data1 holds the source discriminator (FreeRTOS pattern check vs
             * Cortex-M33 hardware STKOF); data2 holds first 4 chars of task
             * name as packed ASCII. */
            char name[5] = {0};
            for (int i = 0; i < 4; i++)
            {
                name[i] = (char)((data2 >> (i * 8)) & 0xFFU);
            }
            if (data1 == STK_OVF_SOURCE_STKOF)
            {
                printf("Type: Stack overflow (hardware STKOF)\n");
            }
            else
            {
                printf("Type: Stack overflow\n");
            }
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
    watchdog_hw->scratch[2] = STK_OVF_SOURCE_FREERTOS;
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

/**
 * @brief Cortex-M HardFault entry point (assembly trampoline).
 *
 * "Trampoline" here means a tiny stub whose only job is to capture some
 * fragile machine state and immediately bounce control on to the real
 * handler - the name comes from that bounce. This stub reads which stack
 * pointer the faulting code was using, then jumps straight into the C
 * handler FaultHandler_HardFaultC(). Keeping it tiny is deliberate: a
 * `naked` function (see below) can hold almost nothing but a branch.
 *
 * Overriding the SDK's weak handler:
 * ----------------------------------
 * The HardFault slot of the Cortex-M vector table is filled by the pico-sdk
 * crt0.S with the symbol `isr_hardfault`, which the SDK declares as a *weak*
 * symbol whose default body is a single `bkpt #0` instruction. Defining our
 * own *strong* (non-weak, external-linkage) function with the exact same name
 * makes the linker bind the vector to us instead - that is the whole override
 * mechanism. Two consequences follow:
 *   - The name must be exactly `isr_hardfault`; rename it and the linker
 *     silently keeps the SDK default (no warning).
 *   - It must not be `static`; internal linkage would disqualify it from
 *     overriding the global weak symbol.
 * Without our override a HardFault hits the SDK's `bkpt #0`: with a debugger
 * attached the core halts into the debugger; with no debugger it escalates to
 * a fault that cannot be taken at HardFault priority, and the CPU enters
 * Lockup - frozen, with no crash diagnostics, until an external reset.
 *
 * Why a "naked" function:
 * -----------------------
 * `naked` (a GCC/Clang ARM function attribute) tells the compiler to emit
 * *only* the body we write - no prologue and no epilogue, at every
 * optimization level, and no implicit return. The body must therefore be
 * pure inline assembly (C in a naked function is undefined - there is no
 * guaranteed stack frame for locals) and must end with its own branch.
 * We need this because on exception entry the hardware places EXC_RETURN in
 * LR, and bit 2 of that value tells us which stack the faulting code used. A
 * normal function's prologue runs *first* and is free to clobber LR / move
 * SP, so by the time any C executed the value would be gone. `naked`
 * guarantees our `tst lr` is the very first instruction to run, with LR
 * still holding the pristine EXC_RETURN.
 * (See GCC docs on "naked" - https://gcc.gnu.org/onlinedocs/gcc-16.1.0/gcc/Common-Attributes.html )
 *
 * What the assembly does:
 * -----------------------
 *
 *      tst   lr, #4              -> test bit 2 of EXC_RETURN held in LR
 *      ite   eq                  -> if/then/else block (Thumb2)
 *      mrseq r0, msp             -> bit 2 clear -> we were on the Main Stack (MSP)
 *      mrsne r0, psp             -> bit 2 set   -> we were on the Process Stack (PSP)
 *      b     FaultHandler_HardFaultC
 *
 * EXC_RETURN bit 2 is the standard Cortex-M indicator of which stack
 * pointer was active at the moment of the exception:
 *   - 0 means the faulting context was using MSP (typically an ISR or
 *     pre-RTOS startup code),
 *   - 1 means it was using PSP (typically a FreeRTOS task).
 *
 * The chosen stack pointer is loaded into r0 - the ARM AAPCS first-argument
 * register - so that the unconditional branch to FaultHandler_HardFaultC()
 * arrives with the correct exception frame pointer in r0. The "b" (not "bl")
 * makes this a tail call: there is nothing to return to anyway because the
 * C handler is noreturn and ends in a watchdog reboot.
 */
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
