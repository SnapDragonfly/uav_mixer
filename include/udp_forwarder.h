#ifndef UDP_FORWARDER_H
#define UDP_FORWARDER_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "config.h"

// Function declarations
int initialize_udp_socket();
void forward_udp_packets(int local_socket);

#endif // UDP_FORWARDER_H
