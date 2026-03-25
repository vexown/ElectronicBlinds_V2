#ifndef WIFI_UDP_H
#define WIFI_UDP_H

bool start_UDP_client(void);
bool udp_client_send(const char* message);
void udp_client_process_recv_message(uint8_t *received_command);

#endif