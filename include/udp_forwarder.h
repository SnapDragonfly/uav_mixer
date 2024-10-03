#ifndef UDP_FORWARDER_H
#define UDP_FORWARDER_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/socket.h>
#include <netinet/in.h>

// UDP configuration
#define LOCAL_PORT 5400
#define FORWARD_IP "192.168.1.19"
#define FORWARD_PORT 5400
#define FORWARD_BUF_LEN 4094

// Function declarations
int initialize_udp_socket();
void forward_udp_packets(int local_socket);

#endif // UDP_FORWARDER_H
