#ifndef UDP_FORWARDER_H
#define UDP_FORWARDER_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "config.h"

// Function declarations
int initialize_udp_socket(uint16_t port);
void get_rtp_data(int local_socket, char *remote_ip, uint16_t remote_port);

#endif // UDP_FORWARDER_H
