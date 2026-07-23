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
#include "task.h"

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

/* How long the chip is left unpowered between the deinit and the rejoin during a
 * radio recovery. cyw43_ensure_up() already holds WL_REG_ON low for 20 ms of its
 * own; this is extra margin for the regulator to fully discharge so the chip
 * comes back from a genuine cold start rather than a marginal brown-out. */
#define RADIO_RECOVERY_OFF_TIME_MS			(250)

/*******************************************************************************/
/*                            STATIC VARIABLES                                 */
/*******************************************************************************/

/* Set by WiFi_RequestRadioRecovery() from the alive task, consumed by
 * WiFi_ServiceRadioRecovery() on the network task. Volatile because it crosses
 * tasks with no lock; single writer per state so no race matters. */
static volatile bool s_radioRecoveryPending = false;

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

        /* Disable wifi power-save. The default CYW43_PERFORMANCE_PM lets the
           chip doze between beacons, so every CYW43 access (including the alive
           LED, which lives on the chip's WL_GPIO0) starts with a wake handshake
           and occasionally stalls for up to the driver's 1 s ioctl timeout.
           This device is mains-powered - keep the chip awake. */
        if (cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM) != 0)
        {
            LOG("failed to disable wifi power-save\n");
        }

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

void WiFi_RequestRadioRecovery(void)
{
    if (!s_radioRecoveryPending)
    {
        LOG("Wi-Fi chip unresponsive - requesting radio power-cycle\n");
        s_radioRecoveryPending = true;
    }
}

bool WiFi_RadioRecoveryPending(void)
{
    return s_radioRecoveryPending;
}

void WiFi_ServiceRadioRecovery(void)
{
    if (!s_radioRecoveryPending)
    {
        return;
    }

    LOG("Recovering Wi-Fi chip: powering down...\n");

    /* Power the chip off and reset all driver state. cyw43_deinit() removes the
     * lwIP netifs, deinits the SPI bus and calls cyw43_init(), which drives
     * WL_REG_ON low - so the radio is genuinely unpowered when this returns.
     *
     * Deliberately NOT cyw43_arch_deinit(): that also tears down the
     * async_context worker task and the lwIP integration. We only need to
     * recycle the driver and the chip, and keeping the surrounding machinery
     * alive makes this far less disruptive to the rest of the system. */
    cyw43_deinit(&cyw43_state);

    vTaskDelay(pdMS_TO_TICKS(RADIO_RECOVERY_OFF_TIME_MS));

    /* Rejoining runs cyw43_ensure_up() internally, which sees the driver marked
     * down, drives WL_REG_ON low->high and re-downloads the chip firmware and
     * CLM blob. That firmware download is why this must go through the driver
     * rather than toggling the pin ourselves - a power-cycled chip comes back
     * with no firmware at all. */
    LOG("Recovering Wi-Fi chip: powering up and rejoining...\n");
    bool reconnected = connectToWifi();

    /* Rebuild the TCP/UDP endpoints either way: the sockets referred to netifs
     * that cyw43_deinit() has removed. If the rejoin failed, the INIT state
     * simply keeps retrying, which is what it already does on a lost link. */
    WiFiState = INIT;

    s_radioRecoveryPending = false;

    if (reconnected)
    {
        LOG("Wi-Fi chip recovered and rejoined.\n");
    }
    else
    {
        /* The chip is alive again even though the network is not - the alive LED
         * will start working, and the normal retry path takes over from here. */
        LOG("Wi-Fi chip power-cycled but rejoin failed - normal retry continues.\n");
    }
}

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
