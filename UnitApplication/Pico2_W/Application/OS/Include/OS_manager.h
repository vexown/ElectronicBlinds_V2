#ifndef OS_MANAGER_H
#define OS_MANAGER_H

#include "FreeRTOS.h"
#include "task.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/* Task-related macros */
#define NUM_OF_TASKS_TO_CREATE 				(4)
#define MAX_NUM_OF_TASKS				    (20)

/*******************************************************************************/
/*                        GLOBAL VARIABLE DECLARATIONS                         */
/*******************************************************************************/
extern TaskHandle_t monitorTaskHandle;

/********************************************************************************/
/*                        GLOBAL FUNCTION DECLARATIONS                          */
/********************************************************************************/
void reset_system(void);
void OS_start(void);


#endif