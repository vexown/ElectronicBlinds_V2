#ifndef COMMON_H
#define COMMON_H

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>

/* Configuration includes */
#include "ModuriConfig.h"

/* WiFi includes */
#include "WiFi_TCP.h"

#ifdef DEBUG_BUILD
#include <stdarg.h>
#endif

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/* Enable printf only in Debug builds */
#ifdef DEBUG_BUILD
    #if (OTA_ENABLED == ON) // During OTA, we don't want to send any debug logs to the OTA server which we will be connected to
        #define LOG printf  // TODO - divide OTA and debug logs into different ports? Connect one client to OTA server and one to debug server?
    #else
        #define LOG(...) (tcp_client_is_connected() ? tcp_send_debug(__VA_ARGS__) : printf(__VA_ARGS__))
    #endif
#else /* RELEASE_BUILD or build type not defined (or defined with invalid type) */
    #define LOG(...)
#endif

/* Error IDs */
#define ERROR_ID_TASK_FAILED_TO_CREATE              (uint8_t)(0x1U)
#define ERROR_ID_WIFI_DID_NOT_CONNECT               (uint8_t)(0x2U)
#define ERROR_ID_RTOS_OBJECTS_FAILED_TO_CREATE      (uint8_t)(0x3U)
#define ERROR_ID_SW_TIMER_FAILED_TO_START           (uint8_t)(0x4U)
#define ERROR_ID_CYW43_INIT_FAILED                  (uint8_t)(0x5U)
#define ERROR_ID_LED_FAILED                         (uint8_t)(0x6U)
#define ERROR_ID_WATCHDOG_RESETS                    (uint8_t)(0x7U)

/* Module IDs */
#define MODULE_ID_OS    (uint8_t)(0x1U)

/* Misc defines */
#define E_OK           0
#define ERRNO_FAIL     -1
#define NO_FLAG        0
#define NON_BLOCKING     0

/*******************************************************************************/
/*                         GLOBAL FUNCTION DECLARATIONS                        */
/*******************************************************************************/

void CriticalErrorHandler(uint8_t moduleId, uint8_t errorId);

#endif