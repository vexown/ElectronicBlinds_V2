#ifndef WIFI_HTTP_H
#define WIFI_HTTP_H

#include <stdint.h>
#include <stdbool.h>
#include "Common.h"
#include "lwip/tcp.h"
#include "WiFi_TCP.h"

#if (HTTP_ENABLED == ON)

#define HTTP_GET "GET"
#define HTTP_RESPONSE_HEADERS "HTTP/1.1 %d OK\nContent-Length: %d\nContent-Type: text/html; charset=utf-8\nConnection: keep-alive\n\n"
#define LED_PARAM "led=%d"
#define LED_TEST "/ledtest"
#define LED_GPIO 0
#define HTTP_RESPONSE_REDIRECT "HTTP/1.1 302 Redirect\nLocation: http://%s" LED_TEST "\n\n"

#define PICO_HTML_PAGE "<!DOCTYPE html>\
<html lang=\"en\">\
<head>\
    <meta charset=\"UTF-8\">\
    <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">\
    <title>Pico W LED Control</title>\
    <style>\
        body {\
            font-family: Arial, sans-serif;\
            background-color: #f0f8ff;\
            color: #333;\
            text-align: center;\
            padding: 20px;\
        }\
        h1 {\
            color: #0066cc;\
        }\
        p {\
            font-size: 18px;\
        }\
        a {\
            display: inline-block;\
            margin-top: 15px;\
            padding: 10px 20px;\
            text-decoration: none;\
            background-color: #0066cc;\
            color: white;\
            border-radius: 5px;\
            font-size: 16px;\
        }\
        a:hover {\
            background-color: #004c99;\
        }\
    </style>\
</head>\
<body>\
    <h1>Welcome to Pico W!</h1>\
    <p>The LED is currently <strong>%s</strong>.</p>\
    <p><a href=\"?led=%d\">Turn LED %s</a></p>\
</body>\
</html>"

/* Sends an HTTP GET request over TCP */
bool send_http_get_request(const char *host, const char *path);

/* Processes the received HTTP response */
void process_HTTP_response(tcpServerType* tcpServer, struct tcp_pcb *pcb);

#endif /* HTTP_ENABLED */

#endif /* WIFI_HTTP_H */