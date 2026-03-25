/**
 * File: WiFi_main.c
 * Description: Implementation of the main WiFi function 
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
#include "WiFi_UDP.h"
#include "WiFi_TCP.h"
#include "WiFi_Common.h"
#include "WiFi_OTA_download.h"
#include "WiFi_OTA_Config.h"

/* OS includes */
#include "OS_manager.h"

/* Flash includes */
#include "flash_layout.h"
#include "flash_operations.h"
#include "metadata.h"

/* Misc includes */
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

/* WiFi Communication Type */
#if (PICO_W_AS_TCP_SERVER == ON)
static TransportLayerType TransportLayer = TCP_COMMUNICATION; /* Force TCP communication type when setting up Pico as TCP server */
#else
static TransportLayerType TransportLayer = TCP_COMMUNICATION; /* select either TCP or UDP */
#endif

/* Current Communication State */
WiFiStateType WiFiState = INIT; 

/*******************************************************************************/
/*                          STATIC FUNCTION DECLARATIONS                       */
/*******************************************************************************/
static void WiFi_ProcessCommand(uint8_t command);
static void WiFi_ListenState(void);
static void WiFi_ActiveState(void);
static void WiFi_InitCommunication(void);
static void WiFi_UpdateState(void);

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/
/* 
 * Function: WiFi_MainFunction
 * 
 * Description: Main implementation of WiFi-related functionalities of the board.
 * It is periodically (every NETWORK_TASK_PERIOD_TICKS) executed from the WiFi 
 * FreeRTOS task
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 *
 */

void WiFi_MainFunction(void)
{
    switch (WiFiState)
    {
        case LISTENING:
            WiFi_ListenState();
            break;
        case ACTIVE_SEND_AND_RECEIVE:
            WiFi_ActiveState();
            break;
        case UPDATE:
            WiFi_UpdateState();
            break;
        default: /* INIT */
            WiFi_InitCommunication();
            break;
    }
}

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: WiFi_ProcessCommand
 * 
 * Description: Processes commands received via TCP or UDP communication from 
 * the Central App. Note - for Pico to process the command correctly, send them 
 * from the Central Application in the following format: "cmd:X" where x = 0..255
 * 
 * Parameters:
 *   - uint8_t command: 0..255 command number (mapped in WiFi_Common.h to a macro)
 * 
 * Returns: void
 *
 */
static void WiFi_ProcessCommand(uint8_t command)
{
    switch (command)
    {
        case PICO_NO_COMMAND:
            break;
        case PICO_HELP:
        {
            LOG("Help command received\n");
            const char *help_msg =
                "Available commands:\n"
                "  cmd:1 - Show this help message\n"
                "  cmd:2 - Transition to Active Mode\n"
                "  cmd:3 - Transition to Listen Mode\n"
                "  cmd:4 - Toggle Monitoring State\n"
                "  cmd:5 - Transition to Update Mode (OTA)\n";
            (void)tcp_send(help_msg, (uint16_t)strlen(help_msg));
            break;
        }
        case PICO_TRANSITION_TO_ACTIVE_MODE:
            LOG("Transitioning to Active Mode...\n");
            WiFiState = ACTIVE_SEND_AND_RECEIVE;
            break;
        case PICO_TRANSITION_TO_LISTEN_MODE:
            LOG("Transitioning to Listen Mode...\n");
            WiFiState = LISTENING;
            break;
        case PICO_TOGGLE_MONITORING_STATE:
            LOG("Pressing on/off button to toggle monitoring...\n");
            xTaskNotifyGive(monitorTaskHandle); // Signal Monitor Task to enable/disable monitoring
            break;
        case PICO_TRANSITION_TO_UPDATE_MODE:
            LOG("Transitioning to Update Mode...\n");
            WiFiState = UPDATE;
            break;
        default:
            LOG("Command not supported \n");
            break;
    }
}

/* 
 * Function: WiFi_ListenState
 * 
 * Description: State in which the Pico W is simply listening to the TCP/IP comms and waits for commands.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 *
 */
static void WiFi_ListenState(void)
{
    uint8_t received_command = PICO_NO_COMMAND;

    if(TransportLayer == TCP_COMMUNICATION)
    {
        /************** RX **************/
        /* Handle any received commands */
        tcp_receive_cmd(&received_command);
        
        WiFi_ProcessCommand(received_command); 
    }
    else if(TransportLayer == UDP_COMMUNICATION)
    {
        /************** RX **************/
        /* Handle any received commands */
        udp_client_process_recv_message(&received_command);

        WiFi_ProcessCommand(received_command); 
    }
    else
    {
        LOG("Communication Type not supported\n");
    }
}

/* 
 * Function: WiFi_ActiveState
 * 
 * Description: State in which the Pico W can both receive/send messages and process 
 * various commands. This state can be achieved by sending PICO_TRANSITION_TO_ACTIVE_MODE 
 * command to take Pico out of the passive LISTENING state.
 * 
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 *
 */
static void WiFi_ActiveState(void)
{
    uint8_t received_command = PICO_NO_COMMAND;
    const char *message = "Yo from Pico W!";

    if(TransportLayer == TCP_COMMUNICATION)
    {
        /************** RX **************/
        /* Handle any received commands */
        tcp_receive_cmd(&received_command);

        WiFi_ProcessCommand(received_command); 

        /************** TX **************/
        /* Send a message (for now just a test message) */
        (void)tcp_send(message, (uint16_t)strlen(message));
    }
    else if(TransportLayer == UDP_COMMUNICATION)
    {
        /************** RX **************/
        /* Handle any received commands */
        udp_client_process_recv_message(&received_command);
        WiFi_ProcessCommand(received_command); 

        /************** TX **************/
        /* Send a message (for now just a test message) */
        udp_client_send(message);
    }
    else
    {
        LOG("Communication Type not supported\n");
    }
}

/* 
 * Function: WiFi_InitCommunication
 * 
 * Description: Initializes the WiFi communication as:
 * - Pico as TCP server (requires PICO_W_AS_TCP_SERVER == ON in ModuriConfig.h)
 * - Pico as TCP client (TransportLayer set to TCP_COMMUNICATION)
 * - Pico as UDP server (currently not supported)
 * - Pico as UDP client (TransportLayer set to UDP_COMMUNICATION)
 * 
 * After initialization the default state of communication is LISTENING.
 * This means the Pico W will passively listen to incoming messages.
 * In this mode it will only print the received messages, and react
 * only to one command - PICO_TRANSITION_TO_ACTIVE_MODE, which transitions
 * the Pico to an active mode.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: void
 *
 */
static void WiFi_InitCommunication(void)
{
    if(TransportLayer == TCP_COMMUNICATION)
    {
#if (PICO_W_AS_TCP_SERVER == ON)
        if(start_TCP_server() == true) WiFiState = LISTENING;
#else /* defaults to Pico as TCP client */
        if(start_TCP_client(REMOTE_TCP_SERVER_IP_ADDRESS, TCP_PORT) == true) WiFiState = LISTENING;
#endif
    }
    else if(TransportLayer == UDP_COMMUNICATION)
    {
        if(start_UDP_client() == true) WiFiState = LISTENING;
    }
    else
    {
        LOG("Communication Type not supported \n");
    }
}

static void WiFi_UpdateState(void)
{
#if (OTA_ENABLED == ON)
    LOG("Initiating firmware download...\n");

    /* Disconnect from the current server */
    tcp_client_disconnect();

    /* Connect to the OTA server */
    bool status = start_TCP_client(OTA_HTTPS_SERVER_IP_ADDRESS, OTA_HTTPS_SERVER_PORT);
    if (!status) 
    {
        LOG("Failed to connect to OTA server\n");
        WiFiState = INIT; // Reinitialize the TCP client in attempt to connect again
        return;
    }
    else
    {
        LOG("Connected to OTA server, attempting to download firmware...\n");
    }

    int download_result = download_firmware();
    
    if (download_result == 0) 
    {
        LOG("Firmware download successful, preparing to apply update\n");
        
        /* Set the update pending flag in metadata */
        boot_metadata_t current_metadata;
        if (read_metadata_from_flash(&current_metadata)) 
        {
            current_metadata.update_pending = true;
            current_metadata.version++; // Increment version for the new firmware
            if (write_metadata_to_flash(&current_metadata)) 
            {
                /* Verify the write by reading back */
                boot_metadata_t verify_metadata;
                read_metadata_from_flash(&verify_metadata);
                LOG("Metadata written to flash: active bank %d, update pending %d\n", verify_metadata.active_bank, verify_metadata.update_pending);
            }
            else
            {
                LOG("Failed to write metadata to flash\n");
            }
        }
        else 
        {
            LOG("Failed to read metadata from flash or it is corrupted\n");
            /* Attempt to recover by erasing the metadata and writing a clean one */
            current_metadata.magic = BOOT_METADATA_MAGIC;
            current_metadata.active_bank = BANK_A;
            current_metadata.version = 0;
            current_metadata.app_size = 0;
            current_metadata.app_crc = 0;
            current_metadata.update_pending = true;
            current_metadata.boot_attempts = 0;

            write_metadata_to_flash(&current_metadata);
        }

        reset_system(); // Reboot the device to apply the update

        while(1)
        { /* should never reach here - update should be applied and device restarted */
            tight_loop_contents(); 
        }
        
    } 
    else if (download_result > 0) 
    {
        LOG("Partial firmware download (%d bytes). Aborting the update. \n", download_result);
        reset_system(); // reset the system to start fresh (I was seeing lwIP issues when attempting to download again without a reset, TODO - investigate)
    } 
    else 
    {
        LOG("Failed to download firmware: error %d\n", download_result);
        reset_system(); // reset the system to start fresh (I was seeing lwIP issues when attempting to download again without a reset, TODO - investigate)
    }
#else
    LOG("OTA is not enabled in the configuration\n");
    WiFiState = LISTENING;
#endif
}



