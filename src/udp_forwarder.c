
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
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
    timeout.tv_sec  = RTP_LOCAL_TO_SEC;
    timeout.tv_usec = RTP_LOCAL_TO_USEC;
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
        static unsigned int packet_count = 1;
        static unsigned int packet_error = 0;
        uint32_t len, num;

        ssize_t recv_len = recvfrom(local_socket, RTP_BUFFER_ADDR(buffer), RTP_BUFFER_SIZE(buffer), 0, NULL, NULL);
        if (recv_len < 0) {
            //perror("Receive failed");

            /*
             * idle UPD time, handling imu sensor data
             * Please try to empty imu ring buffer
             */
            uint32_t offset;
            num = get_rb_count(&g_ring_buff);
            num = (num < RTP_FRAME_IMU_NUM)? num: RTP_FRAME_IMU_NUM;  // max buffer for imu data

            len    = sizeof(mix_head_t);
            offset = sizeof(mix_head_t);
            for (int i = 0; i < num; i++) {
                imu_data_t popped_data;
                (void)pop_rb(&g_ring_buff, &popped_data);

                memcpy(buffer+offset, &popped_data, sizeof(imu_data_t));
                offset += sizeof(imu_data_t);
                len    += sizeof(imu_data_t);
            }
            mix_head_t* p_mixed_head = (mix_head_t*)buffer;
            p_mixed_head->img_nsec = 0;                      // forced, no img time stamp
            p_mixed_head->img_sec  = 0;                      // forced, no img time stamp
            p_mixed_head->imu_num  = num;                    // number of imu data
            p_mixed_head->reserved = MAGIC_IMU_FRAME_NUM;    // magic tag

            if (0 != num) {
                update_rtp_imu_stats(&g_rtp_stats, num);
                ssize_t sent_len = sendto(local_socket, buffer, len, 0, (const struct sockaddr *)&forward_addr, addr_len);
                if (sent_len < 0) {
                    perror("Send failed");
                    break;
                }
            }
            continue;
        }


        /*
         * We got UDP packet from RTP client
         * Now it's crucial for us to do the proper condition tests
         */

        // Check if the received packet is a valid RTP packet
        bool valid = is_valid_rtp_packet((const uint8_t *)RTP_BUFFER_ADDR(buffer), recv_len);
        update_rtp_packet_stats(&g_rtp_stats, valid, recv_len);

        if (valid) {

            /*
             * RTP packet time check
             */
            static struct timespec last_rtp_time = {
                .tv_sec = 0,
                .tv_nsec = 0
            };

            // get latest frame capture time
            struct timespec packet_time;
            if(get_sync_status(&g_sync_time)) {
                (void)estimate_time(&g_sync_time, &packet_time, GET_RTP_TIMESTAMP(buffer));
                //printf("packet time - sec: %ld, nsec: %ld\n", packet_time.tv_sec, packet_time.tv_nsec);
                if (is_before(&packet_time, &last_rtp_time)) {
                    //printf("\033[31mwarning %u time - sec: %ld, nsec: %ld\033[0m\n", GET_RTP_TIMESTAMP(buffer), packet_time.tv_sec, packet_time.tv_nsec);
                    packet_time.tv_sec  = last_rtp_time.tv_sec;
                    packet_time.tv_nsec = last_rtp_time.tv_nsec;
                } else {
#if 0
                    packet_time.tv_sec  = 0;
                    packet_time.tv_nsec = 0;
#else
                    struct timespec current_time;
                    clock_gettime(CLOCK_REALTIME, &current_time);
                    packet_time.tv_sec    = current_time.tv_sec;
                    packet_time.tv_nsec   = current_time.tv_nsec;
                    last_rtp_time.tv_sec  = packet_time.tv_sec;
                    last_rtp_time.tv_nsec = packet_time.tv_nsec;
#endif
                }

                (void)time_minus_us(&packet_time, g_rtp_stats.rtp_max_delivery_per_frame);  // minus rtp_max_delivery_per_frame
            } else {
                packet_time.tv_sec  = 0;
                packet_time.tv_nsec = 0;
            }

            /*
             * Handle IMU data, ahead of real RTP packet
             */
            uint32_t len, num, offset;
            char* p_buffer = NULL;

            num = get_rb_count(&g_ring_buff);
            if (0 != num) {
                num = (num < FORWARD_RTP_IMU_NUM)? num: FORWARD_RTP_IMU_NUM;
                p_buffer = RTP_BUFFER_ADDR(buffer) - sizeof(mix_head_t) - num*sizeof(imu_data_t);

                /*
                 * Worst case assumption
                 */
                (void)time_minus_us(&packet_time, g_rtp_stats.frame_estimate_interval);
                (void)time_minus_us(&packet_time, RTP_FRAME_ADJUST_MS*1000);

                mix_head_t* p_mixed_head = (mix_head_t*)p_buffer;
                p_mixed_head->img_sec  = packet_time.tv_sec;
                p_mixed_head->img_nsec = packet_time.tv_nsec; 
                p_mixed_head->imu_num  = num;
                p_mixed_head->reserved = 0;

                //printf("%u img(%u) sec: %u nsec: %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer),  p_mixed_head->img_sec, p_mixed_head->img_nsec);

                len    = sizeof(mix_head_t);
                offset = sizeof(mix_head_t);
                for (int i = 0; i < num; i++) {
                    imu_data_t popped_data;
                    (void)pop_rb(&g_ring_buff, &popped_data);

                    memcpy(p_buffer+offset, &popped_data, sizeof(imu_data_t));
                    offset += sizeof(imu_data_t);
                    len    += sizeof(imu_data_t);
                }
                update_rtp_imu_stats(&g_rtp_stats, num);
            } else {
                p_buffer = RTP_BUFFER_ADDR(buffer) - sizeof(mix_head_t);
                len = sizeof(mix_head_t);
                memset(p_buffer, 0, sizeof(mix_head_t));
            }

            /*
             * Forward the valid RTP packet
             */
            //ssize_t sent_len = sendto(local_socket, RTP_BUFFER_ADDR(buffer), recv_len, 0, (const struct sockaddr *)&forward_addr, addr_len);
            ssize_t sent_len = sendto(local_socket, p_buffer, recv_len+len, 0, (const struct sockaddr *)&forward_addr, addr_len);
            if (sent_len < 0) {
                perror("Send failed");
                break;
            }

            /*
             * Sync T_m first time, important
             */
            if (packet_count == 1){
                synchronize_time(&g_sync_time, GET_RTP_TIMESTAMP(buffer));
                printf("%u sync first\n", GET_RTP_TIMESTAMP(buffer));
            } 
            packet_count++;

            /*
             * RTP packet stats statistics
             */
            if(is_first_packet_of_frame(GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_MARKER(buffer))){
                update_rtp_head_stats(&g_rtp_stats);
                /*
                 * RTP video timestamp sync
                 */
                unsigned int calculated_timestamp = calculate_timestamp(&g_sync_time);
                int64_t delta_timestamp = (int64_t)(calculated_timestamp*1.0 - (int64_t)GET_RTP_TIMESTAMP(buffer));
                //printf("%lld %u %u\n", delta_timestamp, calculated_timestamp, GET_RTP_TIMESTAMP(buffer));
                if (packet_count % RTP_FRAME_SYNC_NUM == 0) {
                    if ( delta_timestamp < 0 || delta_timestamp > RTP_CLOCK_CTR_THRESHOLD){
                        packet_error++;
                        synchronize_time(&g_sync_time, GET_RTP_TIMESTAMP(buffer));
                        set_sync_status(&g_sync_time, false);
                        printf("\033[1;31m%u sync(%u) delta(%lld) clock(%f)\033[0m\n", 
                               GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), delta_timestamp, g_sync_time.clock_hz);
                    }else{
                        set_sync_status(&g_sync_time, true);
                        //printf("%u mon %u vs %u --> delta %lld\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), (uint32_t)calculated_timestamp, delta_timestamp);
                    }
                }

                if(delta_timestamp < RTP_CLOCK_CTR_MIN_DELAY){
                    inc_sync_clock(&g_sync_time);
                    //printf("%u - timestamp: %u vs %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), calculated_timestamp);
                } else if (delta_timestamp > RTP_CLOCK_CTR_MAX_DELAY) {
                    dec_sync_clock(&g_sync_time);
                    //printf("%u + timestamp: %u vs %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), calculated_timestamp);
                }else {
                    //printf("%u = timestamp: %u vs %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), calculated_timestamp);
                }
                //printf("head-> seq(%u) timestamp: %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer));
            }else{
                //printf("body-> seq(%u) timestamp: %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer));
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
