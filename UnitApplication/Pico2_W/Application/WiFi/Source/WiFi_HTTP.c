/**
 * File: WiFi_HTTP.c
 * Description: High level implementation for HTTP communication over TCP.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

/* WiFi includes */
#include "WiFi_HTTP.h"
#include "WiFi_TCP.h"

/* Misc includes */
#include "Common.h"

/* LED includes */
#include "pico/cyw43_arch.h"

/* Standard includes */
#include <stdio.h>
#include <string.h>

/*******************************************************************************/
/*                               STATIC VARIABLES                              */
/*******************************************************************************/
#if (HTTP_ENABLED == ON)
static char response[2048]; // 2kB response buffer for HTTP GET requests. This will be stored in RAM in the .bss section.
#endif

/*******************************************************************************/
/*                         STATIC FUNCTION DECLARATIONS                        */
/*******************************************************************************/

static int test_server_content(const char *request, const char *query, char *result, size_t max_result_len);

/*******************************************************************************/
/*                      GLOBAL FUNCTION DEFINITIONS                            */
/*******************************************************************************/

#if (HTTP_ENABLED == ON)
/**
 * Function: send_http_get_request
 * 
 * Description: Sends an HTTP GET request to the specified host and path.
 * 
 * Parameters:
 *  - host: The host to send the request to.
 *  - path: The path to send the request to.
 * 
 * Returns: bool indicating the success (true) or failure (false) of sending the HTTP GET request.
 * 
 * Example Usage: 
 * 
 *       const char *server_host = PICO_W_AP_TCP_SERVER_ADDRESS;
 *       const char *pathON = "/ledtest?led=1";
 * 
 *       send_http_get_request(server_host, pathON);
 * 
 */
bool send_http_get_request(const char *host, const char *path) 
{
    // Formulate the HTTP GET request
    char http_request[256];
    int len = snprintf(http_request, sizeof(http_request),
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n\r\n",
        path, host);

    if (len < 0 || len >= sizeof(http_request)) 
    {
        LOG("HTTP request string too long or failed to format.\n");
        return false;
    }

    // Send the HTTP GET request
    err_t result = tcp_send(http_request, len);
    if (result != ERR_OK) 
    {
        LOG("Failed to send HTTP request: %d\n", result);
        return false;
    }

    LOG("HTTP request sent successfully:\n%s\n", http_request);

    return true;
}

/**
 * Function: process_HTTP_response
 * 
 * Description: Processes the received HTTP GET request and generates the response.
 * 
 * Parameters:
 * - tcpServer: pointer to the global TCP server structure
 * - pcb: Pointer to the TCP protocol control block associated with the connection.
 * 
 * Returns: void 
 * 
 * Format of HTTP GET request:
 *      GET /path/resource HTTP/1.1
 *      Host: example.com
 *      [Optional headers]
 */
void process_HTTP_response(tcpServerType* tcpServer, struct tcp_pcb *pcb) 
{
    /* Compare the beginning of the buffer with the HTTP_GET string to check if the buffer contains an HTTP GET request */
    if (strncmp(HTTP_GET, (const char *)tcpServer->receive_buffer, strlen(HTTP_GET)) == 0) // returns 0 if equal
    {
        /* Extract the request from the buffer */
        char *request = (char *)tcpServer->receive_buffer + strlen(HTTP_GET) + 1; // +1 to skip the space after GET
        LOG("Parsed Request: %s\n", request);

        /* Find the '?' character in the request. In the context of an HTTP request, the ? character is used to separate the URL path from the query string. 
           For example HTTP request URL typically looks like this: http://example.com/path/to/resource?key1=value1&key2=value2
           By finding the ? character, you can extract the query string and then parse it to get the individual parameters and their values. */
        char *query = strchr(request, '?');

        if (query) // if the '?' character was found
        {
            /* Find the SECOND space character in the request (Since we skipped the space after GET). For example, in the request 
               "GET /path/resource HTTP/1.1", the second space divides the path/resource from the HTTP version. */
            char *space = strchr(request, ' ');

            /* This line replaces the '?' character we found earlier with a null character (0), effectively terminating the string at that point. 
               This is often done to split a string into two parts. For example, if query is "/ledtest?led=1", this line would change it to 
               /ledtest\0led=1", making "/ledtest" a complete string while moving one character ahead to the query string, which is now "led=1". */
            *query++ = 0;
            LOG("Parsed Query: %s\n", query);

            if (space) // if the second space character was found
            { 
                *space = 0; // replace the second space character with a null character (0) to terminate the string at that point
            }
        }

        /* Generate content */
        int response_len;
        int header_len;
        response_len = test_server_content(request, query, response, sizeof(response));

        LOG("Request: %s?%s\n", request, query);
        LOG("Response length: %d\n", response_len);

        /* Check we had enough buffer space */
        if (response_len > strlen(response)) 
        {
            LOG("Too much response data %d\n", response_len);
            tcp_close_client_connection(pcb);
            return;
        }

        /* Generate web page */
        if (response_len > 0) 
        {
            header_len = snprintf(tcpServer->receive_buffer, sizeof(tcpServer->receive_buffer), HTTP_RESPONSE_HEADERS, 200, response_len);
            if (header_len > strlen(tcpServer->receive_buffer)) 
            {
                LOG("Too much header data %d\n", header_len);
                tcp_close_client_connection(pcb);
                return;
            }
        } 
        else 
        {
            /* Send redirect */
            header_len = snprintf(tcpServer->receive_buffer, sizeof(tcpServer->receive_buffer), HTTP_RESPONSE_REDIRECT, ipaddr_ntoa(&tcpServer->gateway));
            LOG("Sending redirect %s", tcpServer->receive_buffer);
        }

        /* Send the headers to the client */
        err_t err = tcp_write(pcb, tcpServer->receive_buffer, header_len, 0);
        if (err != ERR_OK) 
        {
            LOG("Failed to write header data %d\n", err);
            tcp_close_client_connection(pcb);
            return;
        }

        /* Send the body to the client */
        if (response_len) 
        {
            err = tcp_write(pcb, response, response_len, 0);
            if (err != ERR_OK) 
            {
                LOG("Failed to write response data %d\n", err);
                tcp_close_client_connection(pcb);
                return;
            }
        }
    }

}

/*******************************************************************************/
/*                         STATIC FUNCTION DEFINITIONS                         */
/*******************************************************************************/

static int test_server_content(const char *request, const char *query, char *response, size_t max_response_len) 
{
    int len = 0;
    if (strncmp(request, LED_TEST, strlen(LED_TEST)) == 0) // check if the request is for the LED test query
    {
        /* Get the state of the led */
        bool value;
        cyw43_gpio_get(&cyw43_state, LED_GPIO, &value);
        int led_state = value; 

        /* See if the user changed it in the received HTTP GET request */
        if (query) 
        {
            int led_param = sscanf(query, LED_PARAM, &led_state);

            if (led_param == 1) // led param was found in the query
            {
                if (led_state) 
                {
                    cyw43_gpio_set(&cyw43_state, LED_GPIO, true); // Turn led on
                } 
                else 
                {
                    cyw43_gpio_set(&cyw43_state, LED_GPIO, false); // Turn led off
                }
            }
        }
        /* Generate the response */
        if (led_state) 
        {
            len = snprintf(response, max_response_len, PICO_HTML_PAGE, "ON", 0, "OFF");
        } 
        else 
        {
            len = snprintf(response, max_response_len, PICO_HTML_PAGE, "OFF", 1, "ON");
        }
    }

    if(len > 0)
    {
        LOG("Generated Response: %s\n", response);
    }
    
    return len;
}

#endif /* HTTP_ENABLED */
