/**
 * File: ModuriConfig.h
 * Description: Configuration file for the Moduri project. 
 */

 #ifndef MODURICONFIG_H
 #define MODURICONFIG_H
 
 /* Generic macros for turning features ON or OFF */
 #define ON      1
 #define OFF     0
 
 /*******************************************************************************/
 /*                             COMMUNICATION                                   */
 /*******************************************************************************/
 
 /* This option decides if TCP stack on the Pico is set up as TCP server or TCP client */
 #define PICO_W_AS_TCP_SERVER   OFF
 
 /* This option decides if HTTP is enabled on the Pico, it is then included within the TCP stack */
 #define HTTP_ENABLED           OFF
 
 /* Enable this option if you want to set the IP address of the Pico to a static value PICO_W_STATIC_IP_ADDRESS. Otherwise DHCP is used */
 #define USE_STATIC_IP          ON
 
 /* Use this option if you're running the Pico as an Access Point with a TCP server */
 #define PICO_AS_ACCESS_POINT   OFF
 #if (PICO_AS_ACCESS_POINT == ON)
     #if (PICO_W_AS_TCP_SERVER == OFF) 
     #error "PICO_AS_ACCESS_POINT requires PICO_W_AS_TCP_SERVER to be ON" 
     #endif
     #if (HTTP_ENABLED == OFF) 
     #error "PICO_AS_ACCESS_POINT requires HTTP_ENABLED to be ON" 
     #endif
 #endif
 
 /* Use this option to enable printing (or TCP sending) of monitoring data such as Tasks Statistics, Heap and Stack usage etc. */
 #define MONITORING_ENABLED      OFF
 
 /* Use this option to enable a periodically flashing on-board LED which signalizes the software is running (as opposed to being stuck) */
 #define ALIVE_LED_ENABLED       ON
 
 /* Watchdog timer (WDT) is fundamentally a hardware timer that requires periodic "kicks" or "refreshes" from the software to confirm 
 * that the system is running as expected. If the timer expires (i.e., the software fails to refresh it in time), it triggers a 
 predefined action, typically a system reset (which is our case) */
 #define WATCHDOG_ENABLED        ON
 
 /* Enables the OTA (Over-The-Air) firmware update feature. A TCP client will connect to specified server and download updated firmware if available */
 #define OTA_ENABLED             ON
 #if (OTA_ENABLED == ON)
     #if (PICO_W_AS_TCP_SERVER == ON) 
     #error "OTA_ENABLED requires the Pico to be set up as a TCP client"
     #endif
     #if (USE_STATIC_IP == OFF)
     #error "OTA_ENABLED requires USE_STATIC_IP to be ON"
     #endif
 #endif
 
 #endif // MODURICONFIG_H
  