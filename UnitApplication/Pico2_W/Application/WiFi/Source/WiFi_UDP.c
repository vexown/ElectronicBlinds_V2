/**
 * File: WiFi_UDP.c
 * Description: High level implementation for lwIP-based UDP communication between Unit and Central Application.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "semphr.h"

/* Standard includes */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

/* SDK includes */
#include "lwip/udp.h"
#include "lwip/ip_addr.h"

/* WiFi includes */
#include "WiFi_Common.h"

/* Misc includes */
#include "Common.h"

/*******************************************************************************/
/*                                STATIC VARIABLES                             */
/*******************************************************************************/
static struct udp_pcb *udp_client_pcb;
static uint8_t recv_data[UDP_RECV_BUFFER_SIZE];
static SemaphoreHandle_t bufferMutex;

/*******************************************************************************/
/*                          STATIC FUNCTION DECLARATIONS                       */
/*******************************************************************************/
static void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
static bool udp_send_message(const char *message, const ip_addr_t *dest_addr);
static bool udp_client_init(void);

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: udp_client_process_recv_message
 * 
 * Description: Processes the received UDP message by examining the content of the 
 * receive buffer. The function performs the following tasks:
 * - Acquires a mutex (`bufferMutex`) before accessing the shared receive buffer.
 * - Prints the entire received message.
 * - Checks if the received message starts with the prefix `"cmd:"`. If it does:
 *   - Attempts to extract a numeric value following `"cmd:"` using `atoi`.
 *   - Interprets this number as a command, ensuring it is within the valid range of 0-255.
 *   - If valid, the command value is stored in the provided `received_command` pointer.
 * - If no valid command is found, appropriate messages are logged.
 * - Clears the receive buffer after processing.
 * - Releases the mutex after processing is complete.
 * 
 * Parameters:
 *   - `received_command`: A pointer to a `uint8_t` where the extracted command value will 
 *     be stored if a valid command is found. If no valid command is present, this value 
 *     remains unchanged.
 * 
 * Returns: void
 *
 */
void udp_client_process_recv_message(uint8_t *received_command) 
{
    // Wait for the mutex before accessing the buffer
    if (xSemaphoreTake(bufferMutex, NON_BLOCKING) == pdTRUE) 
    {
        // Access the receive buffer
        uint8_t *buffer = recv_data;

        // Print the entire received buffer
        LOG("Received message: %s\n", buffer);

        // Check if the buffer starts with "cmd:"
        if (strncmp((const char *)buffer, "cmd:", 4) == 0) 
        {
            // Extract the number after "cmd:"
            // Explanation of atoi return value:
            // - If the string after "cmd:" is non-numeric (e.g., "cmd:abc"), atoi will return 0.
            // - If there is nothing after "cmd:" (e.g., "cmd:"), atoi will return 0.
            // - If the string after "cmd:" is explicitly "0", atoi will return 0 as a valid value.
            int command_value = atoi((const char *)&buffer[4]);

            // Ensure the command value is within 0-255 range
            if (command_value >= 0 && command_value <= 255) 
            {
                *received_command = (uint8_t)command_value;
                LOG("Received command: %u\n", *received_command);
            } 
            else 
            {
                LOG("Command value out of range (0-255).\n");
            }
        } 
        else 
        {
            LOG("No command found in received message.\n");
        }

        // Clear the buffer after processing
        memset(recv_data, 0, sizeof(recv_data));

        // Release the mutex after access
        xSemaphoreGive(bufferMutex); 
    } 
    else 
    {
        LOG("Failed to acquire the pointer to receive buffer.\n");
    }
}

/* 
 * Function: start_UDP_client
 * 
 * Description: Initializes the UDP client and sets up synchronization mechanisms for 
 * managing access to shared resources. This function performs the following tasks:
 * - Calls `udp_client_init` to initialize the UDP client.
 * - Logs the result of the UDP client initialization.
 * - Creates a mutex (`bufferMutex`) to protect the shared receive buffer. If the mutex 
 *   creation fails, the function logs an error and updates the status to indicate failure.
 * 
 * Parameters:
 *   - None
 * 
 * Returns: 
 *   - `true` if the UDP client was successfully initialized and the mutex was created.
 *   - `false` if the initialization of the UDP client or the creation of the mutex failed.
 *
 */
bool start_UDP_client(void) 
{
    bool status = udp_client_init();

    if (status == true) 
    {
        LOG("UDP client initialized successfully\n");
    } 
    else 
    {
        LOG("UDP client initialization failed\n");
    }

    bufferMutex = xSemaphoreCreateMutex();
    if (bufferMutex == NULL) 
    {
        LOG("Failed to create mutex\n");
        status = false;
    }
    
    return status;
}

/* 
 * Function: udp_client_send
 * 
 * Description: Sends a UDP message to a pre-defined destination address. The destination 
 * IP address is initialized only once, either by converting a string representation 
 * (`REMOTE_TCP_SERVER_IP_ADDRESS`) to an IP address or by setting it directly to a default value 
 * (192.168.1.194) if the conversion fails. Once the destination address is set, the 
 * function calls `udp_send_message` to send the message.
 * 
 * If the message fails to send, an error message is printed indicating the failure. 
 * The function returns the status of the send operation.
 * 
 * Parameters:
 *   - `message`: A pointer to the message string that needs to be sent via UDP.
 * 
 * Returns: 
 *   - `true` if the message is successfully sent.
 *   - `false` if an error occurs during the send operation.
 *
 */
bool udp_client_send(const char* message) 
{
    static ip_addr_t dest_addr;
    static bool addr_initialized = false;
    
    // Initialize the address only once
    if (!addr_initialized) {
        // Try string conversion first
        if (ipaddr_aton(REMOTE_TCP_SERVER_IP_ADDRESS, &dest_addr) != 1) {
            // If string conversion fails, use direct method
            IP4_ADDR(&dest_addr, 192, 168, 1, 194);
        }
        addr_initialized = true;
    }
    
    // Send the message
    bool result = udp_send_message(message, &dest_addr);
    if (!result) {
        LOG("Failed to send UDP message to %s\n", REMOTE_TCP_SERVER_IP_ADDRESS);
    }
    
    return result;
}

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

/* 
 * Function: udp_receive_callback
 * 
 * Description: A callback function triggered when a UDP packet is received. This function 
 * handles the following tasks:
 * - Checks if the received packet (`pbuf *p`) is not null.
 * - Uses a mutex (`bufferMutex`) to safely access and modify the shared receive buffer.
 * - Clears the receive buffer and copies the packet's payload into the buffer, provided 
 *   the packet size is within the defined `UDP_RECV_BUFFER_SIZE`.
 * - Null-terminates the data if it's being treated as a string.
 * - Logs the received message's source IP address and port.
 * - Frees the allocated packet buffer after processing the received data.
 * 
 * If the mutex cannot be acquired, the function will print an error message and the 
 * packet buffer will still be freed.
 * 
 * Parameters:
 *   - `arg`: A pointer to user-defined data (unused in this implementation).
 *   - `pcb`: A pointer to the UDP Protocol Control Block associated with the connection.
 *   - `p`: A pointer to the received packet buffer (`pbuf`).
 *   - `addr`: A pointer to the source IP address of the received packet.
 *   - `port`: The source port number of the received packet.
 * 
 * Returns: void
 *
 */
static void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) 
{
    /* Unused parameters, but required for the callback signature */
    (void)arg;
    (void)pcb;

    if (p != NULL) 
    {
        // Wait for the mutex before accessing the buffer
        if (xSemaphoreTake(bufferMutex, NON_BLOCKING) == pdTRUE) 
        {
            memset(recv_data, 0, sizeof(recv_data));
        
            if (p->len < UDP_RECV_BUFFER_SIZE) 
            {
                memcpy(recv_data, p->payload, p->len);

                recv_data[p->len] = '\0';  // Null terminate, assuming the received data will be used as string

                LOG("Received UDP message from %s:%d\n", ipaddr_ntoa(addr), port);
            }

            // Release the mutex after finishing with the buffer
            xSemaphoreGive(bufferMutex);
        } 
        else 
        {
            LOG("Failed to take mutex\n");
        }
        
        pbuf_free(p);
    }
}

/* 
 * Function: udp_send_message
 * 
 * Description: Sends a UDP message to the specified destination address. This function 
 * handles the following:
 * - Validates input parameters, ensuring the UDP client Protocol Control Block (PCB), 
 *   message, and destination address are not null.
 * - Allocates a packet buffer (`pbuf`) for the message.
 * - Copies the message content into the allocated buffer.
 * - Sends the UDP packet to the specified destination address and port defined by 
 *   `UDP_SERVER_PORT`.
 * - Frees the packet buffer after sending the message.
 * 
 * If any step fails (e.g., null parameters or buffer allocation failure), the function 
 * will return `false`.
 * 
 * Parameters:
 *   - `message`: A pointer to the message string to be sent.
 *   - `dest_addr`: A pointer to the destination IP address (`ip_addr_t`) to which the 
 *     message should be sent.
 * 
 * Returns: 
 *   - `true` if the message is successfully sent.
 *   - `false` if an error occurs during the process (e.g., null parameters, allocation 
 *     failure, or send error).
 *
 */
static bool udp_send_message(const char *message, const ip_addr_t *dest_addr) 
{
    if (!udp_client_pcb || !message || !dest_addr) {
        return false;
    }
    
    size_t requested_message_len = strlen(message);
    
    // Validate message length fits in uint16_t
    if (requested_message_len > UINT16_MAX) {
        LOG("UDP message too long (%zu bytes, maximum is %u)\n", requested_message_len, UINT16_MAX);
        return false;
    }
    
    uint16_t message_len = (uint16_t)requested_message_len;
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, message_len, PBUF_RAM);
    
    if (!p) {
        return false;
    }
    
    // Copy message to pbuf
    memcpy(p->payload, message, message_len);
    
    // Send the packet
    err_t err = udp_sendto(udp_client_pcb, p, dest_addr, UDP_SERVER_PORT);
    
    // Free the packet buffer
    pbuf_free(p);
    
    return (err == ERR_OK);
}

/* 
 * Function: udp_client_init
 * 
 * Description: Initializes the UDP client for communication. This function performs 
 * the following tasks:
 * - Creates a new UDP Protocol Control Block (PCB).
 * - Binds the UDP client to a local port defined by `UDP_CLIENT_PORT`.
 * - Sets a callback function (`udp_receive_callback`) to handle incoming UDP messages.
 * - Displays network information, including the IP address, netmask, and gateway of the 
 *   default network interface.
 * 
 * If the creation of the UDP PCB or the binding process fails, the function will clean up 
 * and return `false`. Otherwise, it will return `true` upon successful initialization.
 * 
 * Parameters:
 *   - none
 * 
 * Returns: 
 *   - `true` if the initialization is successful.
 *   - `false` if there is an error during the initialization process.
 *
 */
static bool udp_client_init(void) 
{
    // Create new UDP PCB
    udp_client_pcb = udp_new();
    if (!udp_client_pcb) {
        LOG("Failed to create UDP PCB\n");
        return false;
    }
    
    // Bind to local port
    err_t err = udp_bind(udp_client_pcb, IP_ADDR_ANY, UDP_CLIENT_PORT);
    if (err != ERR_OK) {
        LOG("Failed to bind UDP PCB: %d\n", err);
        udp_remove(udp_client_pcb);
        return false;
    }
    
    // Set receive callback
    udp_recv(udp_client_pcb, udp_receive_callback, NULL);
    
    // Print network information
    struct netif *netif = netif_default;
    if (netif != NULL) {
        LOG("\nNetwork Information:\n");
        LOG("IP Address: %s\n", ipaddr_ntoa(&netif->ip_addr));
        LOG("Netmask: %s\n", ipaddr_ntoa(&netif->netmask));
        LOG("Gateway: %s\n", ipaddr_ntoa(&netif->gw));
        LOG("Listening on port: %d\n\n", UDP_CLIENT_PORT);
    }
    
    return true;
}
