/**
 * File: WiFi_TCP.c
 * Description: High level implementation for lwIP-based TCP communication between Unit and Central Application.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "semphr.h"

/* Standard includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <string.h>

/* SDK includes */
#include "pico/cyw43_arch.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>
#include "lwip/pbuf.h"
#include "lwip/err.h"

/* WiFi includes */
#include "WiFi_HTTP.h"
#include "WiFi_DHCP_server.h"
#include "WiFi_DNS_server.h"
#include "WiFi_OTA_download.h"
#include "WiFi_OTA_Config.h"

/* Misc includes */
#include "Common.h"

/*******************************************************************************/
/*                               GLOBAL VARIABLES                              */
/*******************************************************************************/
#if (PICO_W_AS_TCP_SERVER == ON)
tcpServerType *tcpServerGlobal;
#else
TCP_Client_t *clientGlobal;
#endif

/*******************************************************************************/
/*                               STATIC VARIABLES                              */
/*******************************************************************************/

/* Semaphore for notifying the TCP server that data is available */
static SemaphoreHandle_t tcp_data_available_semaphore = NULL;

/* Common mutex for accessing the TCP receive buffer */
static SemaphoreHandle_t bufferMutex;

/* Mutex for sending data */
static SemaphoreHandle_t sendMutex;

/*******************************************************************************/
/*                         STATIC FUNCTION DECLARATIONS                        */
/*******************************************************************************/
#if (PICO_W_AS_TCP_SERVER == ON)
/* Open/close server */
static bool tcp_server_open();
static void tcp_server_close(tcpServerType *tcpServer);
/* Callback functions */
static err_t tcp_server_accept_callback(void *arg, struct tcp_pcb *client_pcb, err_t err);
static err_t tcp_server_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *buffer, err_t err);
static void tcp_server_err_callback(void *arg, err_t err);
/* Send/receive functions */
static void tcp_server_process_recv_message(uint8_t *received_command);
static void tcp_server_send(const char *data, uint16_t length);
#else
/* Initialization functions */
static TCP_Client_t* tcp_client_init(void);
/* Callback functions */
static err_t tcp_client_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static err_t tcp_client_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err);
static void tcp_client_err_callback(void *arg, err_t err);
/* Send/receive functions */
static void tcp_client_process_recv_message(unsigned char* output_buffer, uint16_t* output_buffer_length, uint8_t *received_command);
static err_t tcp_client_send(const void *data, uint16_t length);
/* Helper functions */
static void tcp_client_cleanup(void);
#endif

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

#ifdef DEBUG_BUILD
/**
 * Function: tcp_send_debug
 * 
 * Description: Formats a debug message using printf-style arguments and sends it via TCP.
 * 
 * Parameters:
 *  - const char *format: Printf-style format string
 *  - ...: Variable arguments corresponding to format string
 * 
 * Returns: err_t indicating the result of the send operation
 */
err_t tcp_send_debug(const char* format, ...)
{
    char debug_buffer[256]; /* Fixed size buffer for debug messages */
    va_list args;
    
    /* Add [PICO] prefix to the buffer */
    strcpy(debug_buffer, "[PICO] ");
    size_t prefix_len = strlen(debug_buffer);
    
    /* Format the message with variable arguments */
    va_start(args, format);
    vsnprintf(debug_buffer + prefix_len, sizeof(debug_buffer) - prefix_len - 1, format, args);
    va_end(args);
    
    /* Ensure null termination */
    debug_buffer[sizeof(debug_buffer) - 1] = '\0';
    
    /* Send the formatted message */
    return tcp_send(debug_buffer, (uint16_t)strlen(debug_buffer)); //cast is safe because debug_buffer is fixed size and not bigger than UINT16_MAX
}
#endif

/* 
 * Function: tcp_send
 * 
 * Description: Unified function for sending data over TCP. It calls the appropriate
 *              send function based on the configuration of the Pico as a TCP server
 *              or client.
 * 
 * Parameters:
 *  - const void *data: Pointer to the data to be sent.
 *  - uint16_t length: Length of the data to be sent.
 * 
 * Returns: err_t indicating the result of the send operation.
 */
err_t tcp_send(const void* data, uint16_t length)
{
#if (PICO_W_AS_TCP_SERVER == ON)
    return tcp_server_send(data, length);
#else
    return tcp_client_send(data, length);
#endif
}

/* 
 * Function: tcp_receive_cmd
 * 
 * Description: Unified function for receiving commands over TCP. It calls the appropriate
 *              receive function based on the configuration of the Pico as a TCP server
 *              or client.
 * 
 * Parameters:
 *  - uint8_t *cmd: Pointer to the buffer where the received command will be stored.
 * 
 * Returns: void
 * 
 * Note: This function doesn't actually receive data, it just fetches the data from the buffer. 
 *       The actual data reception is done asynchronously by the lwIP stack and the receive callbacks.
 */
void tcp_receive_cmd(uint8_t* cmd)
{
#if (PICO_W_AS_TCP_SERVER == ON)
    tcp_server_process_recv_message(cmd); // Fetch data received in tcp_server_recv_callback
#else
    unsigned char received_cmd[CMD_MAX_SIZE_BYTES] = {0};
    uint16_t received_cmd_length = sizeof(received_cmd);

    tcp_client_process_recv_message(received_cmd, &received_cmd_length, cmd); // Fetch data received in tcp_client_recv_callback
#endif
}

/* 
 * Function: tcp_receive_data
 * 
 * Description: Unified function for receiving data over TCP. It calls the appropriate
 *              receive function based on the configuration of the Pico as a TCP server
 *              or client.
 * 
 * Parameters:
 *  - unsigned char* buffer: Pointer to the buffer where the received data will be stored.
 *  - uint16_t* buffer_length: Pointer to the length of the received data.
 * 
 * Returns: void
 */
void tcp_receive_data(unsigned char* buffer, uint16_t* buffer_length)
{
#if (PICO_W_AS_TCP_SERVER == ON)
    //tcp_server_process_recv_message(cmd); // Fetch data received in tcp_server_recv_callback - TODO, only receiving commands for now
#else
    uint8_t received_cmd; 

    tcp_client_process_recv_message(buffer, buffer_length, &received_cmd); // Fetch data received in tcp_client_recv_callback

    (void)received_cmd; //unused in the case of receiving data, there is a separate function for receving commands
#endif
}

#if (PICO_W_AS_TCP_SERVER == ON)
/* 
 * Function: start_TCP_server
 * 
 * Description: Starts the TCP server by initializing the tcpServerType structure 
 *              and calling the tcp_server_open function. It handles any necessary 
 *              setup before the server begins accepting connections.
 * 
 * Returns: bool indicating the success (true) or failure (false) of starting the TCP server.
 */
bool start_TCP_server(void) 
{
	bool serverOpenedAndListening = false;
    bool status = false;

	tcpServerGlobal = (tcpServerType*)pvPortCalloc(1, sizeof(tcpServerType)); /* Warning - the server is running forever once started so this memory is not freed */
    if (tcpServerGlobal == NULL)
	{
		LOG("Failed to allocate memory for the TCP server \n");
    }
	else
	{
        ip4_addr_t mask;
        IP4_ADDR(ip_2_ip4(&tcpServerGlobal->gateway), 192, 168, 4, 1); // Set the gateway IP address of the TCP server to 192.168.4.1
        IP4_ADDR(ip_2_ip4(&mask), 255, 255, 255, 0); // Set the subnet mask to 255.255.255.0
        
        dhcp_server_t* dhcp_server = (dhcp_server_t*)pvPortCalloc(1, sizeof(dhcp_server_t));
        if (dhcp_server == NULL) 
        {
            LOG("Failed to allocate memory for DHCP server.\n");
            vPortFree(tcpServerGlobal);
            tcpServerGlobal = NULL;
        }
        /* Start the dhcp server */
        dhcp_server_init(dhcp_server, &tcpServerGlobal->gateway, &mask);

        
        dns_server_t* dns_server = (dns_server_t*)pvPortCalloc(1, sizeof(dns_server_t)); 
        if (dns_server == NULL) 
        {
            LOG("Failed to allocate memory for DNS server.\n");
            vPortFree(tcpServerGlobal);
            vPortFree(dhcp_server);
            tcpServerGlobal = NULL;
            dhcp_server = NULL;
        }
        /* Start the dns server */
        dns_server_init(dns_server, &tcpServerGlobal->gateway);

		serverOpenedAndListening = tcp_server_open();
		if (serverOpenedAndListening == false) 
		{
			LOG("TCP did not successfully open, closing the server and freeing memory... \n");
			tcp_server_close(tcpServerGlobal);
			vPortFree(tcpServerGlobal);
            vPortFree(dhcp_server);
            vPortFree(dns_server);
            tcpServerGlobal = NULL;
            dhcp_server = NULL;
            dns_server = NULL;
		}
		else
		{
			LOG("TCP server started and listening for incoming connections... \n");
			status = true;
		}
	}

    bufferMutex = xSemaphoreCreateMutex();
    if (bufferMutex == NULL) 
    {
        LOG("Failed to create mutex\n");
    }

    return status;
}

#else // PICO_W_AS_TCP_SERVER == OFF

/**
 * Function: tcp_send_mbedtls_callback
 * 
 * Description: Callback function used by mbedTLS to send data over TCP connection.
 *              This function wraps tcp_send() to provide the interface required by mbedTLS.
 * 
 * Parameters:
 *   - shared_context: Unused context pointer that could be used to access TCP client structure
 *   - buf: Pointer to the data buffer to be sent
 *   - len: Length of the data to be sent in bytes
 * 
 * Returns:
 *   - On success: Number of bytes sent (same as len)
 *   - On failure: MBEDTLS_ERR_NET_SEND_FAILED (negative value indicating error)
 * 
 */
int tcp_send_mbedtls_callback(void *shared_context, const unsigned char *buf, size_t len) 
{
    (void)shared_context; // Unused, it can be used for accessing the TCP client structure but we use clientGlobal currently 

    uint16_t buf_length = (uint16_t)len; // Cast to uint16_t, as the send function expects a 16-bit length
    err_t err = tcp_send((const char *)buf, buf_length);
    if (err != ERR_OK) 
    {
        LOG("tcp_send failed: %d\n", err);
        return MBEDTLS_ERR_NET_SEND_FAILED; // To my konwledge, we only need to return x<0 to indicate failure but specific error code will help debugging
    }

    /* Since tcp_send doesn't return bytes sent, assume full send on success. I mention this because the mbedtls_ssl_send_t
       function is expected to return the number of bytes sent. */
    return (int)len;
}

/**
 * Function: tcp_receive_mbedtls_callback
 * 
 * Description: Callback function used by mbedTLS to receive data over TCP connection.
 * 
 * Parameters:
 *  - shared_context: Unused context pointer that could be used to access TCP client structure
 *  - buf: Pointer to the buffer where received data will be stored
 *  - len: Length of the buffer in bytes
 * 
 * Returns:
 * - On success: Number of bytes received (same as len)
 * - On failure: Negative value indicating error (e.g. MBEDTLS_ERR_NET_CONN_RESET, MBEDTLS_ERR_SSL_TIMEOUT)
 *               0 if connection was closed
 */
int tcp_receive_mbedtls_callback(void *shared_context, unsigned char *buf, size_t len) 
{
    (void)shared_context; // Unused, it can be used for accessing the TCP client structure but we use clientGlobal currently

    if(tcp_client_is_connected() == false)
    {
        LOG("Connection to the OTA server is lost\n");
        if((clientGlobal != NULL) && clientGlobal->receive_length > 0) // No connection but there is remaining data in the TCP buffer
        {   
            LOG("Connection is lost but there is remaining data in buffer: %d bytes\n", clientGlobal->receive_length);
        }
        else
        {
            LOG("Connection is lost and the client was deallocated. We cannot process the data anymore\n");
            return MBEDTLS_ERR_NET_CONN_RESET; // Connection reset
        }
    }

    if(clientGlobal->receive_length == 0)
    {
        LOG("No data available in the TCP receive buffer. Waiting for data...\n");

        /* Wait for data notification for up to 5 seconds. Check if the semaphore is available every 100 ms. In the
        meantime, allow other tasks to run. If after 5 seconds no data is available, return a timeout error. */
        TickType_t start_time = xTaskGetTickCount();
        while (xSemaphoreTake(tcp_data_available_semaphore, pdMS_TO_TICKS(100)) != pdTRUE)
        {
            if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(5000)) // Wait for new data for max 5 seconds
            {
                LOG("Timeout waiting for data\n");
                return MBEDTLS_ERR_SSL_TIMEOUT;
            }
        }
        LOG("Data available in the TCP receive buffer\n");

        /* Double check if the connection is still alive */
        if(tcp_client_is_connected() == false)
        {
            LOG("Connection to the OTA server is lost\n");

            /* Ok so we are not connected anymore, but we got here because we received data after waiting for it. So let's process it */
            if((clientGlobal != NULL)) // make sure the client was not deallocated
            {   
                LOG("Connection is lost but there is remaining data in buffer: %d bytes\n", clientGlobal->receive_length);
            }
            else
            {
                LOG("Connection is lost and the client was deallocated. We cannot process the data anymore\n");
                return MBEDTLS_ERR_NET_CONN_RESET; // Connection reset
            }
        }
    }

    /**
     * Maximum fragment length in bytes which determines the size of each of the two internal I/O buffers is 16384 bytes (MBEDTLS_SSL_MAX_CONTENT_LEN).
     * This is due to RFC 5246 (TLS 1.2) and other TLS specs, where the maximum size for a TLS record payload is defined as 2^14 bytes (16,384 bytes).
     * Therefore, our tcp_receive_data function should be able to handle this size (it fits in uint16_t). */
    uint16_t buf_size = (uint16_t)len; // Cast to uint16_t, as the receive function expects a 16-bit length
    tcp_receive_data(buf, &buf_size);

    if(buf_size == len)
    {
        LOG("All data received successfully: %d bytes\n", buf_size);
        LOG("Remaining data in buffer: %d bytes\n", clientGlobal->receive_length);
    }
    else if(buf_size > 0)
    {
        LOG("Partial data received: %d bytes\n", buf_size);
    }
    else
    {
        LOG("No data received\n");
        return MBEDTLS_ERR_SSL_WANT_READ; // No data available, try again
    }

    return (int)buf_size; // Return the number of bytes received
}


/**
 * Function: tcp_client_connect
 * 
 * Description: Connects the TCP client (Pico) to the specified IP address (host) and port of some TCP server.
 * 
 * Parameters:
 * - const char *host: IP address of the server to connect to. Example: "192.168.1.194"
 * - uint16_t port: Port number of the server to connect to. Example: 443 (HTTPS)
 * 
 * Returns: bool indicating the success (true) or failure (false) of the connection.
 */
bool tcp_client_connect(const char *host, uint16_t port) 
{
    ip_addr_t server_ip;

    if(clientGlobal == NULL || clientGlobal->pcb == NULL)
    {
        LOG("TCP client not initialized or PCB is NULL\n");
        WiFiState = INIT; // Reinitialize the TCP client in attempt to connect again
        return false;
    }
    
    /* Check if we are already connected to a host */
    if(tcp_client_is_connected())
    {
        /* Check if trying to connect to a different host/port while already connected */
        ip_addr_t requested_ip;
        if (ipaddr_aton(host, &requested_ip) == 0)
        {
            LOG("Invalid IP address: %s\n", host);
            return false;
        }
        
        /* Compare with current connection */
        ip_addr_t *current_ip = &clientGlobal->pcb->remote_ip;
        u16_t current_port = clientGlobal->pcb->remote_port;
        
        if (!ip_addr_cmp(&requested_ip, current_ip) || port != current_port)
        {
            LOG("Already connected to a different host (%s:%d). Disconnect first before connecting to %s:%d\n",
                ipaddr_ntoa(current_ip), current_port, host, port);
            return false;
        }
        
        LOG("Already connected to requested host\n");
        return true;
    }
    else
    {
        /* The lwIP TCP PCB has a few states. So even if we are not connected, we also need to make sure we are specifically
           in the tcp CLOSED state. It is the only state from which we can attempt to tcp_connect */
        if(clientGlobal->pcb->state == CLOSED)
        {
            LOG("TCP state is CLOSED. Attempting to connect... \n");
        }
        else
        {
            LOG("TCP state is not CLOSED. Current state: %d - Attempting to reinitialize the TCP client... \n", clientGlobal->pcb->state); // find enum tcp_state in pico-sdk for info about the states
            tcp_client_disconnect();
            WiFiState = INIT; // Reinitialize the TCP client in attempt to connect again
            return false;
        }
    }
    
    if (ipaddr_aton(host, &server_ip) == 0) // Convert IP address string (of the host) to numeric value
    {
        LOG("Invalid IP address: %s\n", host);
        return false;
    }
    
    /* Connects the client to the server. Connected callback will be called when the connection is established. */
    /* If connection wasn't properly established, the generic "err" callback will be called (it can be set via tcp_err() function) */
    err_t err = tcp_connect(clientGlobal->pcb, &server_ip, port, tcp_client_connected_callback); // async call, returns immediately
    if (err != ERR_OK) // This DOES NOT check if connection was successful, only if connection request was sent successfully
    {
        LOG("Failed to send connection request: %d\n", err);
        WiFiState = INIT; // Reinitialize the TCP client in attempt to connect again
        return false;
    }

    /* Wait for connection to establish */
    TickType_t xStartTime = xTaskGetTickCount();
    TickType_t xTimeoutTicks = pdMS_TO_TICKS(2000); // 2s

    while (!tcp_client_is_connected() && !clientGlobal->is_closing) 
    {
        /* Check if timeout has occurred */
        if ((xTaskGetTickCount() - xStartTime) > xTimeoutTicks) 
        {
            LOG("Connection timeout\n");
            WiFiState = INIT; // Reinitialize the TCP client in attempt to connect again
            return false;
        }
        
        /* Block this task for 10ms allowing other tasks to run */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    /* Client successfully connected (otherwise the while loop would have returned after timeout) */
    return tcp_client_is_connected();
}

/**
 * Function: tcp_client_disconnect
 * 
 * Description: Disconnects the TCP client from the server and cleans up resources.
 * 
 * Parameters:
 * - void *client: Pointer to the TCP client structure.
 * 
 * Returns: void
 */
void tcp_client_disconnect(void) 
{
    if ((clientGlobal && clientGlobal->pcb) && clientGlobal->is_closing == false) // check if client exists and is not already closing
    {
        clientGlobal->is_closing = true; // indicate that the client is closing the connection

        /* Clear the callbacks for the client PCB to release the associated resources */
        tcp_arg(clientGlobal->pcb, NULL);
        tcp_recv(clientGlobal->pcb, NULL);
        tcp_err(clientGlobal->pcb, NULL);
        tcp_sent(clientGlobal->pcb, NULL);

        /* Close the connection held by the client PCB and free the resources */
        err_t err = tcp_close(clientGlobal->pcb);

        /* According to lwIP documentation in pico-sdk, if tcp_close() returns ERR_OK, the connection was closed successfully
            and if it returns ERR_MEM, the connection was not closed and the application should wait and try again 
            However, in my further testing I have found that tcp_close() returns ERR_OK means that closing of the connection was
            INITIATED, but not necessarily completed yet. To ensure the connection is fully closed (4-way handshake), we need to
            wait for the connection state to be TIME_WAIT or CLOSED. */
        if((err != ERR_OK))
        {
            /* So let's not give up yet and try to initiate the connection closure a few more times */
            TickType_t xStartTime = xTaskGetTickCount();
            TickType_t xTimeoutTicks = pdMS_TO_TICKS(2000); // Try over the next 2 seconds

            while (err != ERR_OK) 
            {
                /* Check if timeout has occurred */
                if ((xTaskGetTickCount() - xStartTime) > xTimeoutTicks) 
                {
                    LOG("Connection close timeout\n");
                    break;
                }
                else if((err = tcp_close(clientGlobal->pcb), err)) /* Try again to close the connection */
                {
                    LOG("Connection closure initiated successfully after a bit of waiting (Ticks waited: %lu)\n", xTaskGetTickCount() - xStartTime);
                    break;
                }
                else /* Block this task for 10ms allowing other tasks to run and try again after */
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        }
        else
        {
            LOG("Connection closure initiated successfully. Client state: %d\n", clientGlobal->pcb->state);
        }

        /* If the connection closure was initiated successfully but is not fully closed yet (4-way handshake not completed) then wait a bit */
        /* TIME_WAIT state (value 10) indicates that the TCP connection's 4-way closing handshake has completed successfully.
        * When a connection enters TIME_WAIT:
        * 1. Both sides have exchanged and acknowledged FIN packets
        * 2. The network-level connection is effectively closed
        * 3. The socket remains in this state for 2MSL (typically 2 minutes) to:
        *    - Handle any delayed duplicate packets
        *    - Ensure reliable delivery of the final ACK
        *    - Prevent confusion between old and new connections
        * 
        * For our purposes, a connection in TIME_WAIT state can be considered successfully closed,
        * even though it hasn't reached the final CLOSED state yet.
        */
        if((err == ERR_OK) && (clientGlobal->pcb->state != CLOSED) && (clientGlobal->pcb->state != TIME_WAIT))
        {
            /* Wait for the connection to complete its handshake */
            TickType_t xStartTime = xTaskGetTickCount();
            TickType_t xTimeoutTicks = pdMS_TO_TICKS(2000); // 2s

            bool is_closed = false;
            while (is_closed != true) 
            {
                /* Check if timeout has occurred */
                if ((xTaskGetTickCount() - xStartTime) > xTimeoutTicks) 
                {
                    LOG("Connection close timeout\n");
                    break;
                }
                else if((clientGlobal->pcb->state == CLOSED) || (clientGlobal->pcb->state == TIME_WAIT)) /* Connection closed successfully */
                {
                    LOG("Connection closed successfully after a bit of waiting (Ticks waited: %lu). Client state: %d\n", xTaskGetTickCount() - xStartTime,
                                                                                                                         clientGlobal->pcb->state);
                    is_closed = true;
                }
                else /* Block this task for 100ms allowing other tasks to run */
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }
        }
        else
        {
            LOG("Connection closed successfully. Client state: %d\n", clientGlobal->pcb->state);
        }

        /* As a last resort, if after the timeout the connection is still not fully closed, 
           force the abort which sends a RST segment to remote host. Ugly but never fails */
        if ((clientGlobal != NULL) && (clientGlobal->pcb != NULL) &&
            (clientGlobal->pcb->state != CLOSED) && (clientGlobal->pcb->state != TIME_WAIT))
        {
            LOG("Error closing TCP connection (Client state: %d) - forcing abort... \n", clientGlobal->pcb->state);

            /* Forcefully abort the connection */
            /* This will send a RST segment to the remote host and free the PCB */
            tcp_abort(clientGlobal->pcb);
            LOG("Connection aborted\n");
        }

        tcp_client_cleanup();
    }
    else if(clientGlobal->is_closing == true)
    {
        LOG("Client is already closing the connection\n");
    }
    else if(clientGlobal != NULL && clientGlobal->pcb == NULL)
    {
        LOG("Client PCB is NULL but client itself is not. Cleaning up...\n");
        tcp_client_cleanup();
    }
    else
    {
        LOG("Client is already closed\n");
    }
}

/* 
 * Function: start_TCP_client
 * 
 * Description: Initializes the TCP client and attempts to connect to the specified server IP address and port.
 * 
 * Parameters:
 *  - const char* host: IP address or hostname of the server to connect to.
 *  - uint16_t port: Port number of the server to connect to.
 * 
 * Returns: bool indicating the success (true) or failure (false) of starting the TCP client.
 */
bool start_TCP_client(const char *host, uint16_t port) 
{
    bool status = true;

    /* Check the initialization and connection status of the TCP client */
    if(clientGlobal != NULL && tcp_client_is_connected())
    {
        LOG("TCP client already started and connected \n");
    }
    else if (clientGlobal != NULL && !tcp_client_is_connected())
    {
        LOG("TCP client is initialized but the connection is not established. Freeing resources and reinitializing... \n");
        status = false;
        
        tcp_client_disconnect(); // Disconnect and free the resources
    }
    else
    {
        LOG("Initializing the TCP client... \n");
        clientGlobal = tcp_client_init();

        if (clientGlobal != NULL) 
        {
            LOG("TCP client initialized successfully\n");
        } 
        else 
        {
            LOG("TCP client initialization failed\n");
            status = false;
        }

        bool connected = tcp_client_connect(host, port);
        if (!connected) 
        {
            LOG("Failed to connect to the TCP server\n");
            WiFiState = INIT; // Reinitialize the TCP client in attempt to connect again
            status = false;
        }
        else
        {
            LOG("Connection to the TCP server established successfully\n");
        }
    }

    /* If the client was successfully initialized and has connected to the server, create the mutexes */
    if(status == true)
    {
        bufferMutex = xSemaphoreCreateMutex();
        if (bufferMutex == NULL) 
        {
            LOG("Failed to create mutex\n");
            status = false;
        }
        sendMutex = xSemaphoreCreateMutex();
        if (sendMutex == NULL) 
        {
            LOG("Failed to create send mutex\n");
            status = false;
        }
    }
        
    return status;
}

/**
 * Function: tcp_client_is_connected
 * 
 * Description: Safely checks if TCP client is connected
 * 
 * Returns: bool indicating connection state
 */
bool tcp_client_is_connected(void) 
{
    if (clientGlobal == NULL || clientGlobal->pcb == NULL) 
    {
        return false;
    }
    
    return (clientGlobal->pcb->state == ESTABLISHED);
}

#endif

/*******************************************************************************/
/*                          STATIC FUNCTION DEFINITIONS                        */
/*******************************************************************************/

#if (PICO_W_AS_TCP_SERVER == ON)
/* 
 * Function: tcp_server_close
 * 
 * Description: Closes the TCP server and releases any associated resources, 
 *              including the client PCB if it exists.
 * 
 * Parameters:
 *  - tcpServer: Pointer to the tcpServerType structure containing server information.
 * 
 * Returns: void
 */
static void tcp_server_close(tcpServerType *tcpServer) 
{
    /* Check if there is an active client PCB. */
    if (tcpServer->client_pcb != NULL) 
    {
        /* Clear the argument associated with the client PCB to detach the server context. */
        tcp_arg(tcpServer->client_pcb, NULL);

        /* Remove the polling function for the client PCB. */
        tcp_poll(tcpServer->client_pcb, NULL, 0);

        /* Clear the receive callback for the client PCB, stopping any data reception. */
        tcp_recv(tcpServer->client_pcb, NULL);

        /* Clear the error callback for the client PCB. */
        tcp_err(tcpServer->client_pcb, NULL);

        /* Close the client PCB to terminate the connection. */
        tcp_close(tcpServer->client_pcb);

        /* Set the client PCB pointer to NULL to indicate no active client. */
        tcpServer->client_pcb = NULL;
    }

    /* Check if there is an active server PCB. */
    if (tcpServer->server_pcb) 
    {
        /* Clear the argument associated with the server PCB to detach the server context. */
        tcp_arg(tcpServer->server_pcb, NULL);

        /* Close the server PCB to stop accepting new connections. */
        tcp_close(tcpServer->server_pcb);

        /* Set the server PCB pointer to NULL to indicate no active server. */
        tcpServer->server_pcb = NULL;
    }
}

/* 
 * Function: tcp_server_recv_callback
 * 
 * Description: Callback function that handles incoming data from the TCP client. 
 *              It processes the received message and notifies lwIP about the 
 *              received data.
 * 
 * Parameters:
 *  - arg: User-defined argument passed to the receive callback, typically containing server context.
 *  - pcb: Pointer to the tcp_pcb structure representing the client connection.
 *  - buffer: Pointer to the pbuf containing the received data.
 *  - err: Error status indicating if there was an error during reception.
 * 
 * Returns: err_t indicating the result of the receive operation.
 */
static err_t tcp_server_recv_callback(void *arg, struct tcp_pcb *pcb, struct pbuf *buffer, err_t err) 
{
    /* Check if the received buffer is NULL. */
    if (!buffer) 
    {
        /* When the client sends a FIN (indicating it wants to close the connection), the tcp_recv callback will be invoked with buffer == NULL. */
        if (err == ERR_OK)
        {
            LOG("Connection closed gracefully\n");
            tcp_close_client_connection(pcb);
            /* err value you return from the tcp_recv callback should guide lwIP in how to proceed with the connection. In this case we tell lwIP all is good */
            return ERR_OK; 
        } 
        else /* In other cases - NULL buffer means something went wrong */
        {
            LOG("Error receiving data: %d\n", err); 
            tcp_close_client_connection(pcb);
            return ERR_ABRT; // Tell lwIP to abort the connection
        }
    }

    if (err != ERR_OK) 
    {
        pbuf_free(buffer);
        return err;
    }

    /* Check if received data fits our buffer and the buffer is not empty */
    if ((buffer->tot_len > TCP_RECV_BUFFER_SIZE) || (buffer->tot_len == 0)) 
    {
        pbuf_free(buffer);
        return ERR_MEM;
    }

    /* Check the lwIP architecture for any state that needs to be updated. */
    cyw43_arch_lwip_check();

    /* Wait for the mutex before accessing the buffer */
    if (xSemaphoreTake(bufferMutex, NON_BLOCKING) == pdTRUE) 
    {
        /* Allocate a buffer to hold the received data plus one byte for the null terminator. */
        /* Copy the received data from the pbuf to the newly allocated buffer. */
        pbuf_copy_partial(buffer, tcpServerGlobal->receive_buffer, buffer->tot_len, 0);

        /* Null-terminate the received string to ensure it's a valid C string. */
        tcpServerGlobal->receive_buffer[buffer->tot_len] = '\0'; 

        /* Print the received message to the console. */
        //LOG("Received message in callback: %s\n", tcpServerGlobal->receive_buffer);

#if (HTTP_ENABLED == ON)
        /* Process the HTTP response */
        process_HTTP_response(tcpServerGlobal, pcb);
#endif
        /* Inform lwIP that the received data has been processed. */
        tcp_recved(pcb, buffer->tot_len);

        /* Release the mutex after finishing with the buffer */
        xSemaphoreGive(bufferMutex);
    } 
    else 
    {
        LOG("Failed to take mutex\n");
    }
    
    /* Free the pbuf to release memory used for the received data. */
    pbuf_free(buffer);

    return ERR_OK;
}

/* 
 * Function: tcp_close_client_connection
 * 
 * Description: Closes the client connection by freeing the resources associated 
 *              with the client PCB and setting the client PCB pointer to NULL.
 * 
 * Parameters:
 *  - client_pcb: Pointer to the tcp_pcb structure representing the client connection.
 * 
 * Returns: void
 */
void tcp_close_client_connection(struct tcp_pcb *client_pcb) 
{
    if (client_pcb) 
    {
        /* Log the client's IP address and port before closing the connection. */
        ip_addr_t *client_ip = &client_pcb->remote_ip;
        u16_t client_port = client_pcb->remote_port;
        LOG("Closing connection for client: IP=%s, Port=%d\n", ipaddr_ntoa(client_ip), client_port);

        /* Clear the callbacks for the client PCB to release the associated resources. */
        tcp_arg(client_pcb, NULL);
        tcp_poll(client_pcb, NULL, 0);
        tcp_sent(client_pcb, NULL);
        tcp_recv(client_pcb, NULL);
        tcp_err(client_pcb, NULL);

        /* Close the client PCB to terminate the connection. */
        err_t err = tcp_close(client_pcb);
        if (err != ERR_OK) 
        {
            LOG("close failed %d, calling abort\n", err);
            tcp_abort(client_pcb);
        }
    } 
    else 
    {
        LOG("tcp_close_client_connection called with NULL client_pcb\n");
    }
}

/* 
 * Function: tcp_server_err_callback
 * 
 * Description: Callback function that is called when an error occurs during 
 *              the TCP server operation. It logs the error and closes the client 
 *              connection if the error is not an abort error.
 * 
 * Parameters:
 *  - arg: User-defined argument passed to the error callback, typically containing server context.
 *  - err: Error status indicating the type of error that occurred.
 * 
 * Returns: void
 */
static void tcp_server_err_callback(void *arg, err_t err) 
{
    (void)arg; // Unused parameter

    if (err != ERR_ABRT) // See list of possible error codes in lwip/err.h in pico-sdk
    {
        LOG("tcp_client_err_fn %d\n", err);
        tcp_close_client_connection(tcpServerGlobal->client_pcb);
    }
}

/* 
 * Function: tcp_server_accept_callback
 * 
 * Description: Callback function that is called when there is a new connection request by a client 
 *              to the TCP server. It sets up the client PCB and prepares to receive data from the client.
 * 
 * Parameters:
 *  - arg: User-defined argument passed to the accept callback, typically containing server context.
 *  - client_pcb: Pointer to the tcp_pcb structure representing the new client connection.
 *  - err: Error status indicating if there was an error during the accept process.
 * 
 * Returns: ERR_OK always - we either accept the connection or abort it.
 */
static err_t tcp_server_accept_callback(void *arg, struct tcp_pcb *client_pcb, err_t err) 
{
    (void)arg; // Unused parameter
    
    /* Check for errors in the accept callback or if the client PCB is NULL. */
    if (err != ERR_OK || client_pcb == NULL) 
    {
        /* Log a failure message if the connection was not successful. */
        if (err != ERR_OK) 
        { 
            LOG("Failure in accept: Error code %d\n", err); 
        } 
        else 
        { 
            LOG("Failure in accept: client_pcb is NULL\n"); 
        }

        if (client_pcb != NULL) 
        {
            tcp_abort(client_pcb); // Clean up the client PCB
        }
    }
	else
	{
		/* Log a message indicating that a client has connected successfully. */
        ip_addr_t *client_ip = &client_pcb->remote_ip;
        u16_t client_port = client_pcb->remote_port;
        LOG("Client connected from IP: %s, Port: %d\n", ipaddr_ntoa(client_ip), client_port);

		/* Store the client PCB in the tcpServer structure for later use. */
		tcpServerGlobal->client_pcb = client_pcb;

		/* Set the receive callback function for the client PCB to handle incoming data. */
		tcp_recv(client_pcb, tcp_server_recv_callback);

        /* Set the error callback function for the client PCB to handle connection errors. */
        tcp_err(client_pcb, tcp_server_err_callback);
	}
    
    return ERR_OK; // Always return ERR_OK to lwIP (we either accept the connection or abort it)
}

/* 
 * Function: tcp_server_open
 * 
 * Description: Initializes and opens the TCP server, binding it to a specified port 
 *              and setting up a listening state for incoming connections.
 * 
 * Parameters: void (uses the global tcpServer structure)
 * 
 * Returns: bool indicating the success (true) or failure (false) of the server opening.
 */
static bool tcp_server_open(void) 
{
    /* Create a new TCP protocol control block (PCB) for the server,
       allowing any IP address type to be used (IPv4 or IPv6). */
    struct tcp_pcb *pcb = tcp_new_ip_type(IPADDR_TYPE_ANY);

    /* Log the starting message with the server's IP address and port. */
#if (HTTP_ENABLED == ON)
    LOG("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_HTTP_PORT);
#else
    LOG("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), TCP_PORT);
#endif

    /* Check if the PCB was successfully created. */
    if (!pcb) 
    {
        /* Log an error message if PCB creation failed. */
        LOG("failed to create pcb\n");
        return false;
    }

    /* Bind the PCB to the specified port. Use NULL to bind to all available interfaces. */
#if (HTTP_ENABLED == ON)
    err_t err = tcp_bind(pcb, IP_ANY_TYPE, TCP_HTTP_PORT);
#else
    err_t err = tcp_bind(pcb, NULL, TCP_PORT);
#endif
    if (err) 
    {
        /* Log an error message if binding to the port failed. */
#if (HTTP_ENABLED == ON)
        LOG("failed to bind to port %u\n", TCP_HTTP_PORT);
#else
        LOG("failed to bind to port %u\n", TCP_PORT);
#endif
        return false;
    }

    /* Set the PCB to listen for incoming connections with a backlog of 1. */
    tcpServerGlobal->server_pcb = tcp_listen_with_backlog(pcb, 1);
    if (!tcpServerGlobal->server_pcb) 
    {
        /* Log an error message if setting the server to listen failed. */
        LOG("failed to listen\n");
        if (pcb) 
        {
            /* Close the PCB if it was successfully created before failing to listen. */
            tcp_close(pcb);
        }
        return false; 
    }

    /* Set the accept callback function for incoming connections. */
    tcp_accept(tcpServerGlobal->server_pcb, tcp_server_accept_callback);

    /* Return true to indicate the server has been successfully opened and is listening for incoming connections */
    return true;
}

/* 
 * Function: tcp_server_process_recv_message
 * 
 * Description: 
 *      Prints the received buffer and analyzes its content.
 *      Checks if the buffer starts with "cmd:".
 *      If it does, it extracts the numeric value following "cmd:"
 *      and stores it as the received command.
 * 
 * Parameters:
 *  - received_command: Pointer to store the extracted command value
 * 
 * Returns: 
 *  - void
 */
static void tcp_server_process_recv_message(uint8_t *received_command) 
{
    // Wait for the mutex before accessing the buffer
    if (xSemaphoreTake(bufferMutex, NON_BLOCKING) == pdTRUE) 
    {
        // Access the receive buffer
        uint8_t *buffer = tcpServerGlobal->receive_buffer;

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
        memset(tcpServerGlobal->receive_buffer, 0, sizeof(tcpServerGlobal->receive_buffer));

        // Release the mutex after access
        xSemaphoreGive(bufferMutex); 
    } 
    else 
    {
        LOG("Failed to acquire the pointer to receive buffer.\n");
    }
}

/* 
 * Function: tcp_server_send
 * 
 * Description: 
 *      Checks if the TCP server has an active client connection. If connected, 
 *      it uses tcp_write() to queue the data for sending and then uses 
 *      tcp_output() to actually send the queued data over the network.
 * 
 * Parameters:
 *  - const char *data: Pointer to the data buffer to be sent.
 *  - uint16_t length: Length of the data to be sent.
 * 
 * Returns: 
 *  - err_t: An error code indicating the success (ERR_OK) or failure 
 *            (e.g., ERR_CONN if no client is connected or other 
 *            error codes from tcp_write() or tcp_output()) of the send operation.
 */
static err_t tcp_server_send(const char *data, uint16_t length) 
{
    err_t err = ERR_OK;
    
    /* Check if tcpServerGlobal exists and has an active client connection */
    if (tcpServerGlobal == NULL || tcpServerGlobal->client_pcb == NULL) 
    {
        LOG("Cannot send - no client connected\n");
        return ERR_CONN;
    }
    
    /* Send data */
    err = tcp_write(tcpServerGlobal->client_pcb, data, length, TCP_WRITE_FLAG_COPY);
    if (err == ERR_OK) 
    {
        /* Actually send the data */
        err = tcp_output(tcpServerGlobal->client_pcb);
        if (err != ERR_OK) 
        {
            LOG("tcp_output failed: %d\n", err);
        }
    } 
    else 
    {
        LOG("tcp_write failed: %d\n", err);
    }
    
    return err;
}

#else // PICO_W_AS_TCP_SERVER == OFF

/* 
 * Function: tcp_client_init
 * 
 * Description: 
 *      Allocates and initializes the TCP client structure, creates a new 
 *      TCP protocol control block (PCB), and initiates a connection to the 
 *      specified server. This function prepares the client for communication 
 *      by setting up necessary callbacks and connecting to the server.
 * 
 * Parameters:
 *  - None
 * 
 * Returns: 
 *  - TCP_Client_t*: A pointer to the initialized TCP client structure if 
 *                   successful or NULL if initialization or connection 
 *                   fails.
 */
static TCP_Client_t* tcp_client_init(void) 
{
    /* Allocate memory for the TCP client structure on the heap */
    TCP_Client_t *client = (TCP_Client_t*)pvPortCalloc(1, sizeof(TCP_Client_t));

    /* Set flow control parameters */
    client->flow_control_threshold = (TCP_RECV_BUFFER_SIZE / 2 ); // 50% of the buffer size
    client->flow_throttled = false;

    /* Create a binary semaphore for signaling data availability */
    tcp_data_available_semaphore = xSemaphoreCreateBinary();

    if (client == NULL) 
    {
        LOG("Failed to allocate client structure\n");
    }
    else if (tcp_data_available_semaphore == NULL) 
    {
        LOG("Failed to create semaphore\n");
        vPortFree(client);
        client = NULL;
    }
    else
    {
        /* Create new TCP PCB (Protocol Control Block) for the client */
        /* PCB is a central data structure for managing an active TCP connection. Each TCP connection is represented by one tcp_pcb instance, 
         * which contains all the necessary information. This structure is manipulated by the TCP stack to implement the protocol according to 
         * RFC 793 and its extensions. It maintains all necessary state to handle the TCP connection lifecycle from establishment to termination. */
        client->pcb = tcp_new();
        if (client->pcb == NULL) 
        {
            LOG("Failed to create TCP PCB\n");
            vPortFree(client);
            return NULL;
        }

        /* Set client structure as an argument for callback functions (meaning that the callbacks will have access to the client structure) */
        tcp_arg(client->pcb, client);

        /* Set up error callback */
        tcp_err(client->pcb, tcp_client_err_callback);
    }

    return client;
}

/* 
 * Function: tcp_client_send
 * 
 * Description: 
 *      Checks if the TCP client is connected. If connected, it uses tcp_write() 
 *      to queue the data for sending and then uses tcp_output() to actually send 
 *      the queued data over the network.
 * 
 * Parameters:
 *  - const void *data: Pointer to the data buffer to be sent.
 *  - uint16_t length: Length of the data to be sent.
 * 
 * Returns: 
 *  - err_t: An error code indicating the success (ERR_OK) or failure 
 *            (e.g., ERR_CONN if the client is not connected or other 
 *            error codes from tcp_write() or tcp_output()) of the send operation.
 */
static err_t tcp_client_send(const void *data, uint16_t length) 
{
    err_t err = ERR_OK;
    
    /* Check if clientGlobal and its PCB exist and we are connected */
    if (clientGlobal == NULL || !tcp_client_is_connected() || clientGlobal->pcb == NULL) 
    {
        LOG("Cannot send - client not connected. Please connect first.\n"); // You can try reconnecting via tcp_client_connect() when handling this error
        err = ERR_CONN;
    }
    else
    {
        /* Use a mutex to protect the send operation (only one task can send data at a time) */
        if (xSemaphoreTake(sendMutex, pdMS_TO_TICKS(1000)) == pdTRUE) 
        {
            /* Write data for sending (but does not send it immediately) */
            /* It waits in the expectation of more data being sent soon (as it can send them more efficiently by combining them together).
                To prompt the system to send data now, call tcp_output() after calling tcp_write(). Use TCP_WRITE_FLAG_MORE if you want to 
                send more data in the same segment. TCP_WRITE_FLAG_COPY indicates that new memory should be allocated for the data to be copied into. */
            err = tcp_write(clientGlobal->pcb, data, length, TCP_WRITE_FLAG_COPY);
            if (err == ERR_OK) 
            {
                /* Actually send the data */
                err = tcp_output(clientGlobal->pcb);
                if (err != ERR_OK) 
                {
                    LOG("tcp_output failed: %d\n", err);
                }
        
            } else if (err == ERR_MEM) 
            {
                LOG("tcp_write failed with ERR_MEM - length of data exceeds the current send buffer size\n");
            }
            else 
            {
                LOG("tcp_write failed: %d\n", err);
            }   

            /* Release the mutex after sending */
            xSemaphoreGive(sendMutex); 
        }
        else 
        {
            LOG("Failed to acquire the mutex for sending data.\n");
            err = ERR_TIMEOUT; // Indicate that the send operation timed out trying to acquire the mutex
        }
    }
      
    return err;
}

/* 
 * Function: tcp_client_process_recv_message
 * 
 * Description: 
 *      Prints the received buffer and analyzes its content.
 *      Checks if the buffer starts with "cmd:".
 *      If it does, it extracts the numeric value following "cmd:"
 *      and stores it as the received command.
 * 
 * Parameters:
 *  - char* output_buffer: Pointer to the buffer where the received data will be stored.
 *  - uint16_t* output_buffer_length: Pointer to the length of the received data.
 *  - uint8_t* received_command: Pointer to store the extracted command value.
 * 
 * Returns: 
 *  - void
 */
static void tcp_client_process_recv_message(unsigned char* output_buffer, uint16_t* output_buffer_length, uint8_t *received_command)
{
    /* Initialize output parameters to default values */
    *received_command = PICO_NO_COMMAND;
    *output_buffer = '\0';

    /* Make sure we have a valid client and receive buffer */
    if (clientGlobal == NULL || output_buffer == NULL || output_buffer_length == NULL || received_command == NULL) 
    {
        LOG("Invalid client or receive buffer\n");
        WiFiState = INIT; // Reinitialize the TCP client
        return;
    }

    /* Make sure client is connected so we can tell the difference between no connection and no data */
    if (!tcp_client_is_connected()) 
    {
        if(clientGlobal->receive_length == 0)
        {
            LOG("No data received and client is not connected. Trying to reconnect...\n");
#if (OTA_ENABLED == ON)
            (void)tcp_client_connect(OTA_HTTPS_SERVER_IP_ADDRESS, OTA_HTTPS_SERVER_PORT);
#else
            (void)tcp_client_connect(REMOTE_TCP_SERVER_IP_ADDRESS, TCP_PORT);
#endif
            return; // There is no data so nothing more to do. We try to reconnect but no further processing is needed, regardless of the result.
        }
        else
        {
            LOG("Client is not connected but there is data in the receive buffer. Processing it...\n");
        }
    }

    /* Wait for the mutex before accessing the buffer */
    if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(1000)) == pdTRUE) 
    {
        /* Check if the output buffer is large enough to store the received data */
        if(clientGlobal->receive_length > *output_buffer_length)
        {
            LOG("Output buffer is too small to store the received data. Not all data will be read. Call this function again to process the next batch.\n");
            memcpy(output_buffer, clientGlobal->receive_buffer, *output_buffer_length); // Copy only the data that fits in the output buffer

            /* Remove ONLY the data that was copied to the output buffer from the receive buffer */
            uint16_t remaining_data_length = clientGlobal->receive_length - *output_buffer_length;

            /* Move the remaining data to the beginning of the receive buffer */
            memmove(clientGlobal->receive_buffer, &clientGlobal->receive_buffer[*output_buffer_length], remaining_data_length);

            /* Clear the rest of the receive buffer */
            memset(&clientGlobal->receive_buffer[remaining_data_length], 0, sizeof(clientGlobal->receive_buffer) - remaining_data_length);

            /* Update the receive length to reflect the remaining data */
            clientGlobal->receive_length = remaining_data_length;
        }
        else
        {
            *output_buffer_length = clientGlobal->receive_length; // Update the output buffer length with the actual received data length

            /* Copy the received data from the TCP receive_buffer to the output buffer */
            memcpy(output_buffer, clientGlobal->receive_buffer, *output_buffer_length);

            /* Clear the buffer after processing */
            memset(clientGlobal->receive_buffer, 0, sizeof(clientGlobal->receive_buffer));

            /* Set the receive length to 0 */
            clientGlobal->receive_length = 0;
        }

        /* Check if the message starts with "cmd:" and if it's even long enough to contain a command */
        if (*output_buffer_length >= CMD_MIN_SIZE_BYTES && strncmp((const char *)output_buffer, "cmd:", 4) == 0) 
        {
            /* Extract the number after "cmd:"
             * Explanation of atoi return value:
             * - If the string after "cmd:" is non-numeric (e.g., "cmd:abc"), atoi will return 0.
             * - If there is nothing after "cmd:" (e.g., "cmd:"), atoi will return 0.
             * - If the string after "cmd:" is explicitly "0", atoi will return 0 as a valid value.
             */
            int command_value = atoi((const char *)&output_buffer[4]);

            /* Ensure the command value is within 1-255 range */
            if (command_value > 0 && command_value <= 255) 
            {
                *received_command = (uint8_t)command_value;
                LOG("Received command: %u\n", *received_command);
            } 
            else 
            {
                received_command = 0;
                LOG("Command value out of range (1-255).\n");
            }
        }
        else /* Received message is not a command */
        {
            /* Print the entire received buffer (if not empty) */
            if(strlen((const char *)output_buffer) > 0)
            {
                LOG("Data has been stored in the output buffer\n");
            }
        }
        
        /* After processing, update flow control status */
        if (clientGlobal->flow_throttled && 
            clientGlobal->receive_length < (clientGlobal->flow_control_threshold / 2)) 
        {
            /* Buffer has emptied enough, resume normal flow */
            clientGlobal->flow_throttled = false;
            
            /* Send window update to server to resume normal flow */
            /* This increases the advertised window size */
            if (tcp_client_is_connected()) 
            {
                /* When you call `tcp_recved(pcb, len)`, you're saying: "I've processed `len` bytes, please increase my receive window by that amount." 
                    Normally, you would call this function in the tcp_recv callback after processing the received data. However, in this case, we are calling 
                    it here to indicate that the client has recovered from the throttling condition and is ready to receive more data.
                    When you call `tcp_recved(pcb, TCP_RECV_BUFFER_SIZE - receive_length)`, you're essentially resetting the window to match your current 
                    buffer availability rather than incrementally updating it. This is a bit of a hack, but it works for our use case. (TODO - find out if this is the right way to do it) */
                tcp_recved(clientGlobal->pcb, ((uint16_t)TCP_RECV_BUFFER_SIZE - clientGlobal->receive_length));

                LOG("Flow control: resuming normal flow, buffer now %d/%d bytes\n", clientGlobal->receive_length, TCP_RECV_BUFFER_SIZE);
            }
        }
        
        /* Release the mutex after access */
        xSemaphoreGive(bufferMutex); 
    } 
    else 
    {
        LOG("Failed to acquire the mutex for the receive buffer.\n");
    }
}

/* 
 * Function: tcp_client_recv_callback
 * 
 * Description: 
 *      This callback function is called automatically when data arrives 
 *      at the TCP client. It handles the incoming data by storing it in 
 *      the client's buffer, prints the received data (for testing), and 
 *      sends an acknowledgment back to the server. Look for TCP_EVENT_RECV
 *      in the lwIP TCP code/documentation for more info on how this callback
 *      is used.
 * 
 * Parameters:
 *  - void *arg: Additional argument passed to the callback, configured
 *               via tcp_arg() for the given PCB. In this case, it is the
 *               pointer to the TCP_Client_t structure (not really used rn).
 *  - struct tcp_pcb *tpcb: Pointer to the TCP protocol control block 
 *                          associated with the connection that received the data.
 *  - struct pbuf *rcv_data: Pointer to the pbuf structure containing the 
 *                           received data (or NULL if the connection is closed).
 *  - err_t err: As of today's state of lwIP, this is always ERR_OK (0)
 * 
 * Returns: 
 *  - err_t: For TCP receive callbacks (tcp_recv_fn), you should almost always return `ERR_OK` (0) 
 *           which indicates normal processing. This tells lwIP you've handled the data correctly 
 *           (by copying it, processing it, or discarding it) and want the connection to continue. 
 *           You must properly free the pbuf with `pbuf_free()` before returning, unless you're storing 
 *           it for later use (rare). The only exception is when you've called `tcp_abort()` within your 
 *           callback to forcibly terminate the connection - in that specific case, return `ERR_ABRT`. 
 */
static err_t tcp_client_recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *rcv_data, err_t err) 
{
    (void)arg; // Unused parameter, we use the global client structure (clientGlobal)
    (void)err; // This is always ERR_OK (0) so we don't need to check it (as of now)

    if (rcv_data == NULL) // Special condtion where connection has been closed
    {
        LOG("Connection closed by remote host\n");
    }
    else // We have received data - process it
    {
        if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(1000)) == pdTRUE) // make sure noone else is accessing the buffer right now
        {
            uint16_t occupied_buffer_size = clientGlobal->receive_length;
            uint16_t received_data_size = rcv_data->tot_len;
            if ((occupied_buffer_size + received_data_size) <= TCP_RECV_BUFFER_SIZE) // Check if the received data fits in the remaining buffer
            {
                /* Copy the received data from the pbuf to the receive buffer */
                uint8_t* next_free_buffer_position = &clientGlobal->receive_buffer[clientGlobal->receive_length];
                pbuf_copy_partial(rcv_data, next_free_buffer_position, received_data_size, 0 /* offset */);

                /* Update the total length of received data */
                clientGlobal->receive_length += received_data_size;
                
                LOG("Appended %d bytes to receive_buffer, total %d\n", rcv_data->tot_len, clientGlobal->receive_length);
                
                /* Signal that data is available */
                xSemaphoreGive(tcp_data_available_semaphore);

                /* Flow control based on buffer occupancy */
                if (clientGlobal->receive_length >= clientGlobal->flow_control_threshold) 
                {
                    /* Buffer is getting full - throttle the flow */
                    if (!clientGlobal->flow_throttled) 
                    {
                        LOG("Buffer filling up, throttling flow (%d/%d bytes)\n", clientGlobal->receive_length, TCP_RECV_BUFFER_SIZE);

                        clientGlobal->flow_throttled = true;
                    }
                    
                    /* Options to impose reduced flow on the sender:
                    * 1. Acknowledge only a portion of the received data
                    * 2. Don't acknowledge at all (zero window)
                    * 3. Acknowledge but at a reduced rate
                    * 
                    * Here we are using option 2 - deliberately NOT calling tcp_recved(), which creates a "zero window condition"
                    * 
                    * TCP Zero Window Explanation:
                    * - In TCP, the "receive window" advertises how many more bytes a receiver can accept
                    * - When a receiver processes data, it calls tcp_recved() to increase this window
                    * - If the receiver STOPS acknowledging data (by not calling tcp_recved()), the sender's
                    *   usable window gradually shrinks to zero as more unacknowledged data arrives
                    * - Once the window reaches zero, TCP protocol prohibits the sender from transmitting ANY
                    *   more data (except "window probe" packets)
                    * - This creates a complete pause in data transmission - a "zero window condition"
                    * 
                    * Implementation:
                    * - When buffer occupancy exceeds threshold -> we stop calling tcp_recved()
                    * - This naturally leads to a zero window as more data arrives
                    * - Once flow_throttled=true, we maintain zero window until buffer is processed
                    * 
                    * Recovery:
                    * - When buffer drops below threshold -> we call tcp_recved() with available space
                    * - This reopens the window and allows sender to resume transmission
                    * - See tcp_client_process_recv_message() for recovery logic
                    *
                    * Advantages: Simple to implement, very effective at stopping transmission
                    * Disadvantages: Binary on/off control rather than gradual slowdown
                    */
                } 
                else 
                {
                    /* Buffer has space - normal flow */
                    if (clientGlobal->flow_throttled) {
                        LOG("Resuming normal flow, buffer: %d/%d bytes\n", 
                            clientGlobal->receive_length, TCP_RECV_BUFFER_SIZE);
                        clientGlobal->flow_throttled = false;
                    }
                    /* Inform lwIP that the received data of size rcv_data->tot_len has been processed */
                    tcp_recved(tpcb, received_data_size);  // Normal acknowledgment
                }

            } 
            else // There is not enough space in the receive buffer
            {
                LOG("[CRITICAL] Receive buffer overflow: %d + %d > %d\n", clientGlobal->receive_length, rcv_data->tot_len, TCP_RECV_BUFFER_SIZE);
                
                /* Buffer full - set zero window to stop sender completely */
                clientGlobal->flow_throttled = true;
                /* Don't call tcp_recved() here to advertise zero window */
                
            }
        
            /* Let other tasks access the buffer */
            xSemaphoreGive(bufferMutex);
        }
        else 
        {
            LOG("[CRITICAL] Failed to acquire mutex after 1s - network congestion or deadlock detected\n");
        }
        
        /* Free the pbuf to release memory used for receiving data  This is important to avoid memory leaks. 
        The pbuf is no longer needed after processing */
        pbuf_free(rcv_data);
    }

    return ERR_OK; // For now we always return ERR_OK (0) to indicate that we have processed the data correctly
}

/* 
 * Function: tcp_client_connected_callback
 * 
 * Description: 
 *      This callback function is called when a connection to the server is 
 *      established. It sets up the receive callback for handling incoming 
 *      data and updates the connection status of the TCP client.
 * 
 * Parameters:
 *  - void *arg: A pointer to the TCP client structure (TCP_Client_t) that 
 *                is passed when the callback is registered.
 *  - struct tcp_pcb *tpcb: Pointer to the TCP protocol control block 
 *                            associated with the connection.
 *  - err_t err: An error code indicating the result of the connection attempt.
 * 
 * Returns: 
 *  - err_t: An error code indicating the success (ERR_OK) or failure 
 *            (e.g., other error codes if the connection attempt failed).
 */
static err_t tcp_client_connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err) 
{
    (void)arg; // Client structure not used currently in this callback
    
    if (err != ERR_OK) // Connection failed (Host unreachable, Connection refused, etc.)
    {
        LOG("TCP client connection failed: %d\n", err);
        return err;
    }
    
    /* Set up receive callback */
    tcp_recv(tpcb, tcp_client_recv_callback);
    
    LOG("TCP client connected to server\n");
    return ERR_OK;
}

/**
 * Function: tcp_client_err_callback
 * 
 * Description: Callback function that is called when when a fatal error
 *              has occurred on the connection.
 * 
 * Parameters:
 *  - void *arg: A pointer to the TCP client structure (TCP_Client_t) that
 *               is passed when the callback is registered.
 *  - err_t err: Error status indicating the type of error that occurred.
 * 
 * Returns: void
 * 
 * Note: The corresponding pcb is already freed when this callback is called!
 */
static void tcp_client_err_callback(void *arg, err_t err) 
{
    (void)arg; // Unused parameter, we know the client is global

    if(clientGlobal != NULL)
    {
        clientGlobal->is_closing = false;
        clientGlobal->pcb = NULL;
    }

    switch (err) 
    {
        case ERR_ABRT:
            LOG("Connection aborted locally\n");
            break;
        case ERR_RST:
            LOG("Connection reset by remote host\n");
            break;
        case ERR_CLSD:
            LOG("Connection closed by remote host\n");
            break;
        case ERR_TIMEOUT:
            LOG("Connection timed out\n");
            break;
        default:
            LOG("TCP Client Error: %d\n", err);
            break;
    }

    /* Attempt to recover connection by reinitializing the client */
    WiFiState = INIT;
}

/**
 * Function: tcp_client_cleanup
 * 
 * Description: Cleans up TCP client resources by resetting state flags,
 *              freeing memory, and nullifying pointers.
 * 
 * Parameters: None - operates on the global client structure
 * 
 * Returns: void
 */
static void tcp_client_cleanup(void)
{
    if (clientGlobal != NULL)
    {
        clientGlobal->pcb = NULL;
        /* PCB is already freed in tcp_close() function, but we do need to free the clientGlobal structure manually */
        if(clientGlobal != NULL)
        {
            vPortFree(clientGlobal);
            clientGlobal = NULL;
            LOG("TCP client resources cleaned up\n");
        }
        else
        {
            LOG("TCP client already deallocated\n");
        }

        /* Setting clientGlobal->is_closing=false is not done here since we are deallocating the clientGlobal structure */
    }
}

#endif

