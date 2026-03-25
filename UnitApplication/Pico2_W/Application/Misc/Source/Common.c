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

/* Misc includes */
#include "Common.h"

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
    LOG("CRTICIAL ERROR OCCURED. moduleId: %u, errorId: %u. Going into endless loop...\n", moduleId, errorId);

    /* Enter a critical section. The idea here is to prevent leaving this endless loop by disabling all interrupts globally.
    As stated in FreeRTOS documentation:
        Preemptive context switches only occur inside an interrupt, so will not occur when interrupts are disabled. Therefore, the 
        task that called taskENTER_CRITICAL() is guaranteed to remain in the Running state until the critical section is exited, 
        unless the task explicitly attempts to block or yield (which it should not do from inside a critical section). */ 
    taskENTER_CRITICAL();
    
    /* Infinite loop to trap execution */
    while(1)
    {
        /* TODO - You can also add some flashing LEDs or other indicators  */
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
    /* Inside this function, pcFile holds the name of the source file that
    contains the line that detected the error, and ulLine holds the line
    number in the source file. The pcFile and ulLine values can be printed
    out, or otherwise recorded, before the following infinite loop is
    entered. (TODO, for now just printing but consider other ways of sending/storing this error info) */
    printf("Assertion failed in file: %s at line: %u\n", pcFile, (unsigned int)ulLine);

    /* Disable interrupts so the tick interrupt stops executing, then sit in a
    loop so execution does not move past the line that failed the assertion. */
    taskDISABLE_INTERRUPTS();
    for( ;; );
}


