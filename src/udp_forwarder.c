
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>

#include "udp_forwarder.h"
#include "rtp_statistics.h"  

extern volatile sig_atomic_t running;
rtp_stats_t stats;

int initialize_udp_socket() {
    int sockfd;
    struct sockaddr_in local_addr;

    // Create a UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("Socket creation failed");
        exit(EXIT_FAILURE);
    }

    // Bind the socket to the local port
    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(LOCAL_PORT);

    if (bind(sockfd, (const struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        perror("Bind failed");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // Set receive timeout for the socket
    struct timeval timeout;
    timeout.tv_sec = 2;  // 2 seconds timeout
    timeout.tv_usec = 0; // 0 microseconds timeout
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("Failed to set socket options");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    init_rtp_stats(&stats); // Initialize RTP statistics

    return sockfd;
}

void forward_udp_packets(int local_socket) {
    char buffer[FORWARD_BUF_LEN];
    struct sockaddr_in forward_addr;
    socklen_t addr_len = sizeof(forward_addr);

    // Set up the address to forward to
    memset(&forward_addr, 0, sizeof(forward_addr));
    forward_addr.sin_family = AF_INET;
    forward_addr.sin_port = htons(FORWARD_PORT);
    if (inet_pton(AF_INET, FORWARD_IP, &forward_addr.sin_addr) <= 0) {
        perror("Invalid address for forwarding");
        close(local_socket);
        exit(EXIT_FAILURE);
    }

    while (running) {
        ssize_t recv_len = recvfrom(local_socket, buffer, sizeof(buffer), 0, NULL, NULL);
        if (recv_len < 0) {
            perror("Receive failed");
            continue;
        }
        update_rtp_recv_len(&stats, recv_len);

        // Check if the received packet is a valid RTP packet
        bool valid = is_valid_rtp_packet((const uint8_t *)buffer, recv_len);
        update_rtp_stats(&stats, valid);

        if (valid) {
            // Forward the valid RTP packet
            ssize_t sent_len = sendto(local_socket, buffer, recv_len, 0, (const struct sockaddr *)&forward_addr, addr_len);
            if (sent_len < 0) {
                perror("Send failed");
                break;
            }

            if(1 == GET_RTP_MARKER(buffer)){
                update_rtp_head_stats(&stats);
            }else{
                update_rtp_body_stats(&stats); 
            }
        } else {
            printf("Invalid RTP packet received. Total invalid count: %u\n", stats.invalid_count);
        }
    }

    // Print RTP statistics
    print_rtp_stats(&stats);
}
