/**
 * File: WiFi_Init.c
 * Description: Anything related to initialization and config of the WiFi connection
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"

/* SDK includes */
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

/* WiFi includes */
#include "WiFi_Credentials.h" //Create WiFi_Credentials.h with your WiFi login and password as const char* variables called ssid and pass
#include "WiFi_Common.h"

/* Misc includes */
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/* WiFi macros */
#define WIFI_CONNECTION_TIMEOUT_MS 			(10000) //10s 

/*******************************************************************************/
/*                         STATIC FUNCTION DECLARATIONS                          */
/*******************************************************************************/
static void configStaticIP(void);

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/
/* 
 * Function: connectToWifi
 * 
 * Description: Function connects to the WiFi network specified in the WiFi_Credentials.h file
 * 
 * Parameters:
 *   - none
 * 
 * Returns: bool
 *  - true if connection successful
 *  - false if connection failed for any reason
 *
 */

bool connectToWifi(void)
{
    bool status = false;

    /* Enables Wi-Fi in Station (STA) mode such that connections can be made to other Wi-Fi Access Points */
    cyw43_arch_enable_sta_mode();

    /* Attempt to connect to a wireless access point (currently my Tenda WiFi router)
       Blocking until the network is joined, a failure is detected or a timeout occurs */
    LOG("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, WIFI_CONNECTION_TIMEOUT_MS)) 
    {
        LOG("failed to connect\n");
    }
    else
    {
        LOG("connected sucessfully\n");
        status = true;
#if (USE_STATIC_IP == ON)
        configStaticIP();
#endif
    }

    return status;
}

/**
 * Function: setupWifiAccessPoint
 * 
 * Description: Function sets up the Pico W as a WiFi Access Point
 * 
 * Parameters:
 *  - none
 * 
 * Returns: bool
 * - always returns true for now
 * 
 */

bool setupWifiAccessPoint(void)
{
    const char* ssid_local = "MainBox";
    const char* pass_local = "password"; //THIS IS NOT A SECRET, ONLY FOR TESTING 

    /* Enables Wi-Fi in Access Point (AP) mode with WPA2 security */
    cyw43_arch_enable_ap_mode(ssid_local, pass_local, CYW43_AUTH_WPA2_AES_PSK);

    return true; 
}

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/
#if (USE_STATIC_IP == ON)
/* 
 * Function: configStaticIP
 * 
 * Description: Configures the IP address of the Pico W to a static address specified
 * by PICO_W_STATIC_IP_ADDRESS macro
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 */
static void configStaticIP(void)
{
    ip4_addr_t ipaddr, netmask, gateway;
	struct netif *netif;

    /* Set the Pico W's network interface's to specific static address, netmask, and gateway */
	netif = netif_default;
    ipaddr.addr = ipaddr_addr(PICO_W_STATIC_IP_ADDRESS);
    netmask.addr = ipaddr_addr(NETMASK_ADDR);
    gateway.addr = ipaddr_addr(GATEWAY_ADDR);
    netif_set_addr(netif, &ipaddr, &netmask, &gateway);
}
#endif // USE_STATIC_IP
