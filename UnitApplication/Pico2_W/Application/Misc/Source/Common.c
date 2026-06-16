/**
 * File: Common.c
 * Description: Various project-wide common functions, types, macros etc. Not specific to any SWC.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Standard includes. */
#include <stdio.h>

/* Kernel includes */
#include "FreeRTOS.h"
#include "task.h"

/* Pico SDK includes */
#include "pico/stdlib.h"        /* busy_wait_ms() */
#include "hardware/watchdog.h"  /* watchdog_update() */

/* Misc includes */
#include "Common.h"
#include "FaultHandler.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* Parked-state loop timing. We feed the watchdog every CRIT_PARK_TICK_MS, which
 * MUST stay below WATCHDOG_TIMEOUT_MS (2000 ms) so the board stays parked instead
 * of reset-looping, while still being rescued if this very loop ever wedges.
 * The reason line is re-emitted every CRIT_PARK_REPRINT_TICKS ticks (~5 s) so a
 * UART serial adapter connected at any later time immediately sees what happened. */
#define CRIT_PARK_TICK_MS           ((uint32_t)250U)
#define CRIT_PARK_REPRINT_TICKS     ((uint32_t)20U)   /* 20 * 250 ms = ~5 s */

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: CriticalErrorHandler
 * 
 * Description:  Function to trap execution and with an error code
 * 
 * Parameters:
 *   - moduleId: id of the module (SWC) where the error happened
 *   - errorId: id of the specific error
 * 
 * Returns: void
 */
void CriticalErrorHandler(uint8_t moduleId, uint8_t errorId)
{
    uint32_t ticks = 0;

    for ( ;; )
    {
#if (WATCHDOG_ENABLED == ON)
        watchdog_update();
#endif
        if ((ticks % CRIT_PARK_REPRINT_TICKS) == 0U)
        {
            printf("\n[CRITICAL] PARKED - moduleId=%u errorId=%u - Power-cycle or reflash to recover.\n",
                  (unsigned)moduleId, (unsigned)errorId);
        }

        ticks++;
        busy_wait_ms(CRIT_PARK_TICK_MS);
    }
}

/* 
 * Function: vAssertCalled 
 * 
 * Description: Custom function to be called with configASSERT macro to log errors in terminal.
 * 
 * Parameters:
 *   - *pcFile - pointer to the source file in which the assert was triggered
 *   - ulLine -  line number on which the assert was triggered in the mentioned source file
 * 
 * Returns: void
 */
void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
    /* Record file/line into watchdog scratch regs, log, then reboot. After the
     * reboot FaultHandler_ReportLastCrash() prints the saved info so a crash
     * leaves a trail even if nothing was watching the terminal at the time. */
    FaultHandler_RecordAssert(pcFile, ulLine);
}


