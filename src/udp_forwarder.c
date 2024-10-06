
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>

#include "udp_forwarder.h"
#include "rtp_statistics.h"
#include "time_sync.h"
#include "ring_buffer.h"

extern volatile sig_atomic_t running;
extern rtp_stats_t g_rtp_stats;
extern sync_time_t g_sync_time;
extern ring_buffer_t g_ring_buff;

#define TIMING_STATUS(A, B)  ((A <= B)?"OK":"NG")

#define RTP_BUFFER_ADDR(buffer) (buffer+FORWARD_RTP_PREFIX_LEN)
#define RTP_BUFFER_SIZE(buffer) (sizeof(buffer)-FORWARD_RTP_PREFIX_LEN)

// Grab marker bit from RTP buffer, indicating first RTP packet of a frame
#define GET_RTP_MARKER(data)  (((RTP_BUFFER_ADDR(data))[1] >> 7) & 0x01)

// Macro to extract the 16-bit sequence number from the RTP buffer
#define GET_RTP_SEQUENCE_NUMBER(data) (((uint16_t)(RTP_BUFFER_ADDR(data)[2]) << 8) | (uint16_t)(RTP_BUFFER_ADDR(data)[3]))

// Macro to extract the 32-bit timestamp from the RTP buffer
#define GET_RTP_TIMESTAMP(data) (((uint32_t)(RTP_BUFFER_ADDR(data)[4]) << 24) | ((uint32_t)(RTP_BUFFER_ADDR(data)[5]) << 16) | ((uint32_t)(RTP_BUFFER_ADDR(data)[6]) << 8) | (uint32_t)(RTP_BUFFER_ADDR(data)[7]))

int initialize_udp_socket(uint16_t port) {
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
    local_addr.sin_port = htons(port);

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

    return sockfd;
}

void forward_udp_packets(int local_socket, char *remote_ip, uint16_t remote_port) {
    char buffer[FORWARD_BUF_LEN];
    struct sockaddr_in forward_addr;
    socklen_t addr_len = sizeof(forward_addr);

    // Set up the address to forward to
    memset(&forward_addr, 0, sizeof(forward_addr));
    forward_addr.sin_family = AF_INET;
    forward_addr.sin_port = htons(remote_port);
    if (inet_pton(AF_INET, remote_ip, &forward_addr.sin_addr) <= 0) {
        perror("Invalid address for forwarding");
        close(local_socket);
        exit(EXIT_FAILURE);
    }

    while (running) {
        static unsigned int packet_count = 0;
        static unsigned int packet_error = 0;
        static unsigned int update_count = RTP_FPS_RATE/2;
        static unsigned int check_skip = 0;
        static double previous_error = 100;
        static double latest_error   = 100;
        double error;

        ssize_t recv_len = recvfrom(local_socket, RTP_BUFFER_ADDR(buffer), RTP_BUFFER_SIZE(buffer), 0, NULL, NULL);
        if (recv_len < 0) {
            perror("Receive failed");
            continue;
        }
        update_rtp_recv_len(&g_rtp_stats, recv_len);

        // Check if the received packet is a valid RTP packet
        bool valid = is_valid_rtp_packet((const uint8_t *)RTP_BUFFER_ADDR(buffer), recv_len);
        update_rtp_packet_stats(&g_rtp_stats, valid);

        if (valid) {
            double packet_time;
            if(get_sync_status(&g_sync_time)) {
                packet_time = estimate_time(&g_sync_time, GET_RTP_TIMESTAMP(buffer)) - g_rtp_stats.frame_estimate_interval - RTP_FRAME_ADJUST_MS*1000000;
            } else {
                packet_time = 0;
            }

            imu_data_t popped_data;
            if (pop_rb(&g_ring_buff, &popped_data)) {
                popped_data.img_sec  = (uint32_t)(packet_time / 1e6);
                popped_data.img_nsec = (uint32_t)((packet_time - (popped_data.img_sec * 1e6)) * 1e3); 
                memcpy(buffer, &popped_data, sizeof(imu_data_t));

                //printf("Popped: img_sec=%u, img_nsec=%u\n", popped_data.img_sec, popped_data.img_nsec);
                //printf("Popped: imu_sec=%u, imu_nsec=%u\n", popped_data.imu_sec, popped_data.imu_nsec);
            } else {
                memset(buffer, 0, sizeof(imu_data_t));
            }

            /*
             * Forward the valid RTP packet
             */
            //ssize_t sent_len = sendto(local_socket, RTP_BUFFER_ADDR(buffer), recv_len, 0, (const struct sockaddr *)&forward_addr, addr_len);
            ssize_t sent_len = sendto(local_socket, buffer, recv_len+sizeof(imu_data_t), 0, (const struct sockaddr *)&forward_addr, addr_len);
            if (sent_len < 0) {
                perror("Send failed");
                break;
            }

            /*
             * RTP packet stats statistics
             */
            if(1 == GET_RTP_MARKER(buffer)){
                update_rtp_head_stats(&g_rtp_stats);
                /*
                 * RTP video timestamp sync
                 */
                if (packet_count == 0){
                    synchronize_time(&g_sync_time, GET_RTP_TIMESTAMP(buffer));
                    check_skip = 1;

#if (UAV_MIXER_DEBUG)
                    printf("%u sync first\n", GET_RTP_TIMESTAMP(buffer));
#endif
                } else if (packet_count % RTP_FRAME_SYNC_NUM == 0) {
                    error = calculate_error(&g_sync_time, GET_RTP_TIMESTAMP(buffer));

                    bool status1, status2, status_trend;
                    status_trend = latest_error > previous_error;  // and error's trend is getting large

                    status1   = abs(error) > RTP_FRAME_SYNC_THRESHOLD && status_trend;      // abs() > error threshold
                    status2   = error < 0 && status_trend;                                  // error < 0

                    if ( status1 || status2 ){
                        synchronize_time(&g_sync_time, GET_RTP_TIMESTAMP(buffer));
                        set_sync_status(&g_sync_time, false);
                        check_skip = 1;
                        printf("\033[1;31m%u sync error %.2f %.2f %.2f\033[0m\n", GET_RTP_TIMESTAMP(buffer), error, previous_error, latest_error);
                    }else{
                        set_sync_status(&g_sync_time, true);
                        printf("%u sync skip %.2f %.2f %.2f\n", GET_RTP_TIMESTAMP(buffer), error, previous_error, latest_error);
                    }
                }
                packet_count++;
                update_count++;

                unsigned int calculated_timestamp = calculate_timestamp(&g_sync_time);
                if(GET_RTP_TIMESTAMP(buffer) > calculated_timestamp){
                    packet_error++;
                    previous_error = latest_error;
                    latest_error = 100.0*packet_error/packet_count;
#if (UAV_MIXER_DEBUG)
                        printf("%u error timestamp: %u vs %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), calculated_timestamp);
                }else{
                        printf("%u good timestamp: %u vs %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), calculated_timestamp);
#endif
                }

                if (update_count % RTP_FPS_UPDATE_RATE == 0){
                    if (check_skip == 0){
#if (UAV_MIXER_DEBUG)
                        double estimated_time = estimate_time(&g_sync_time, GET_RTP_TIMESTAMP(buffer));
                        double system_time    = get_system_time_us();
                        // Estimate system time for the latest count
                        printf("%u time %s %.2f vs %.2f µs\n", 
                                GET_RTP_SEQUENCE_NUMBER(buffer), TIMING_STATUS(estimated_time, system_time), estimated_time, system_time);
                        printf("%u stmp %s %u vs %u counts\n", 
                                GET_RTP_SEQUENCE_NUMBER(buffer), TIMING_STATUS(GET_RTP_TIMESTAMP(buffer), calculated_timestamp), GET_RTP_TIMESTAMP(buffer), calculated_timestamp);
#endif
                        previous_error = latest_error;
                        latest_error = 100.0*packet_error/packet_count;
                        printf("%u estim %.2f %% -> %d %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), latest_error, packet_count, calculated_timestamp - GET_RTP_TIMESTAMP(buffer));
                    }else{ 
                        check_skip = 0;
                    }
                }
            }else{
                update_rtp_body_stats(&g_rtp_stats); 
            }
        } else {
            printf("Invalid RTP packet received. Total invalid count: %u\n", g_rtp_stats.invalid_count);
        }
    }

    // Print RTP statistics
    print_rtp_stats(&g_rtp_stats);
    print_rb_stats(&g_ring_buff);
}
