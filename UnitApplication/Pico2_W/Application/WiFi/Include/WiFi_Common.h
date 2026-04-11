#ifndef WIFI_COMMON_H
#define WIFI_COMMON_H

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/

/* Server defines */
#define REMOTE_TCP_SERVER_IP_ADDRESS "192.168.50.194"  //Either UDP or TCP external server IP address that we communicate with (e.g PC, another Pico), statically configured on that server
#define PICO_W_STATIC_IP_ADDRESS     "192.168.50.50"   //Static IP address for the Pico W
#define NETMASK_ADDR                 "255.255.255.0"  //Subnet mask for the network (meaning you can use 192.168.50.1 to 192.168.50.254 for devices
                                                      //(the 0 address is the network address and 255 is the broadcast address).
#define GATEWAY_ADDR                "192.168.50.1"    //Gateway IP address (in this case the Tenda Wifi router is the gateway so it's its' address)
#define SERVER_PORT                 (uint16_t)12345  //TODO - server port for testing, later you can think about which one to use permanently
#define TCP_PORT                    8080
#define TCP_HTTP_PORT               80
#define TCP_RECV_BUFFER_SIZE        65535             //in bytes
#define UDP_SERVER_PORT             5000
#define UDP_CLIENT_PORT             5001
#define UDP_RECV_BUFFER_SIZE        1024

/* OTA server config (IP, port, certificate) is in WiFi_OTA_Config.h */

/* Commands from PC to Pico */
#define CMD_MIN_SIZE_BYTES              5  // "cmd:X" where X is a number from 0-9 (e.g "cmd:1" - 5 bytes, without the null terminator)
#define CMD_MAX_SIZE_BYTES              8  // "cmd:XXX" where 255 is the largest number that can be sent (e.g "cmd:255" - 8 bytes, without null terminator)
#define PICO_NO_COMMAND                 0xFF // Sentinel: no command received, do nothing
#define PICO_HELP                       1
#define PICO_TRANSITION_TO_ACTIVE_MODE  2
#define PICO_TRANSITION_TO_LISTEN_MODE  3
#define PICO_TOGGLE_MONITORING_STATE    4
#define PICO_TRANSITION_TO_UPDATE_MODE  5
#define PICO_SHOW_FW_VERSION            6

/*******************************************************************************/
/*                               DATA TYPES                                   */
/*******************************************************************************/

/* Wifi Communication Types */
typedef enum {
    UDP_COMMUNICATION = 0xAA,
    TCP_COMMUNICATION = 0xAB
} TransportLayerType;

/* Communication State */
typedef enum {
    INIT = 0,
    LISTENING = 1,
    ACTIVE_SEND_AND_RECEIVE = 2,
    UPDATE = 3
} WiFiStateType;

/*******************************************************************************/
/*                          GLOBAL FUNCTION DECLARATIONS                      */
/*******************************************************************************/

/* Non-specific functions */
bool connectToWifi(void);
bool setupWifiAccessPoint(void);
void WiFi_MainFunction(void);

/*******************************************************************************/
/*                             GLOBAL VARIABLES                               */
/*******************************************************************************/
extern WiFiStateType WiFiState;

#endif
