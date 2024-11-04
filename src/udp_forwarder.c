
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <arpa/inet.h>

#include "udp_forwarder.h"
#include "rtp_statistics.h"
#include "time_sync.h"
#include "ring_buffer.h"

extern volatile sig_atomic_t running;
extern rtp_stats_t g_rtp_stats;
extern sync_time_t g_sync_time;
extern ring_buffer_t g_ring_buff;
extern uav_config_t g_uav_config;

struct timespec lastest_rtp_time = {
    .tv_sec = 0,
    .tv_nsec = 0
};
struct timespec previous_rtp_time = {
    .tv_sec = 0,
    .tv_nsec = 0
};

struct timespec lastest_img_time = {
    .tv_sec = 0,
    .tv_nsec = 0
};
struct timespec previous_img_time = {
    .tv_sec = 0,
    .tv_nsec = 0
};

int64_t previous_rtp_seqence_id = 0;
int64_t previous_rtp_timestamp  = 0;


#define TIMING_STATUS(A, B)  ((A <= B)?"OK":"NG")

#define RTP_BUFFER_ADDR(buffer) (buffer+FORWARD_RTP_MIX_LEN)
#define RTP_BUFFER_SIZE(buffer) (sizeof(buffer)-FORWARD_RTP_MIX_LEN)

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
    timeout.tv_usec = g_uav_config.rtp_local_timeout;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        perror("Failed to set socket timeout");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    struct timeval timeout_check;
    socklen_t len = sizeof(timeout_check);
    getsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout_check, &len);
    printf("Expect timeout: %ld s, %ld us\n", timeout.tv_sec, timeout.tv_usec);
    printf("Socket timeout: %ld s, %ld us\n", timeout_check.tv_sec, timeout_check.tv_usec);

    socklen_t optlen;
    int actual_recv_bufsize, actual_send_bufsize;

    // Set receive buffer size
    int recv_bufsize = FORWARD_RECV_BUF_LEN;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &recv_bufsize, sizeof(recv_bufsize))) {
        perror("Failed to set socket receive buffer size");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    // Set send buffer size
    int send_bufsize = FORWARD_SEND_BUF_LEN;  // Define your send buffer size as SEND_BUF_LEN
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &send_bufsize, sizeof(send_bufsize)) < 0) {
        perror("Failed to set socket send buffer size");
        close(sockfd);
        exit(EXIT_FAILURE);
    }

    optlen = sizeof(actual_recv_bufsize);
    if (getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &actual_recv_bufsize, &optlen) < 0) {
        perror("Failed to get socket receive buffer size");
    } else {
        printf("Current receive buffer size: %d bytes\n", actual_recv_bufsize);
    }

    optlen = sizeof(actual_send_bufsize);
    if (getsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &actual_send_bufsize, &optlen) < 0) {
        perror("Failed to get socket send buffer size");
    } else {
        printf("Current send buffer size: %d bytes\n", actual_send_bufsize);
    }

#if 0
    // Set the socket to non-blocking mode
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
        perror("Failed to get socket flags");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
    if (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) < 0) {
        perror("Failed to set socket to non-blocking mode");
        close(sockfd);
        exit(EXIT_FAILURE);
    }
#endif
    return sockfd;
}

int get_send_buffer_usage(int sockfd) {
    int bytes_in_buffer = 0;
    if (ioctl(sockfd, TIOCOUTQ, &bytes_in_buffer) < 0) {
        perror("Failed to get send buffer usage");
    }
    return bytes_in_buffer;
}

int get_recv_buffer_usage(int sockfd) {
    int bytes_in_buffer = 0;
    if (ioctl(sockfd, FIONREAD, &bytes_in_buffer) < 0) {
        perror("Failed to get buffer usage");
    }
    return bytes_in_buffer;
}

void forward_udp_packets(int local_socket, const void *buf, size_t len, char mark, char *remote_ip, uint16_t remote_port) {
    struct timespec p1, p2;
    clock_gettime(CLOCK_REALTIME, &p1);

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


    ssize_t sent_len = sendto(local_socket, buf, len, 0, (const struct sockaddr *)&forward_addr, addr_len);

    int bytes_for_send_buffer = get_send_buffer_usage(local_socket);
    update_rtp_send_buffer_size(&g_rtp_stats, bytes_for_send_buffer);
    if (sent_len < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            printf("Send buffer full(%d), try again later\n", bytes_for_send_buffer);
        } else {
            perror("Send IMU failed");
        }
    } else {
        putchar(mark);
        fflush(stdout);
    }
    clock_gettime(CLOCK_REALTIME, &p2);
    update_rtp_send_time(&g_rtp_stats, timespec_diff_us(&p1, &p2));
}

/*
 * Procedure A: Only IMU data
 * There are amount of IMU data, but RTP transfer is limited.
 * When data number is large than RTP_FRAME_IMU_MIN_NUM, then make the UDP transfer.
 */
char* pack_a_udp_packet(char *buffer, size_t *len) {
    struct timespec p1, p2;
    clock_gettime(CLOCK_REALTIME, &p1);

    uint32_t offset;

    uint32_t num_adjust = 0;
    uint32_t num = get_rb_count(&g_ring_buff);
    num = (num < RTP_FRAME_IMU_NUM)? num: RTP_FRAME_IMU_NUM;  // max buffer for imu data

    if (num < RTP_FRAME_IMU_MIN_NUM) {
        return NULL;
    }

    *len   = sizeof(mix_head_t);
    offset = sizeof(mix_head_t);
    for (int i = 0; i < num; i++) {
        (void)pop_rb(&g_ring_buff, (imu_data_t*)(buffer+offset));

        bool is_before = is_imu_before((imu_data_t*)(buffer+offset), &previous_img_time);
        if (is_before) {
            num_adjust++;
        } else {
            offset += sizeof(imu_data_t);
            *len   += sizeof(imu_data_t);
        }
    }
    num -= num_adjust;

    if(0 == num) { //NO IMU data
        return NULL;
    }

    mix_head_t* p_mixed_head = (mix_head_t*)buffer;
    p_mixed_head->timestamp.base_sec  = 0;           // forced, no time stamp
    p_mixed_head->timestamp.base_sec  = 0;           // forced, no time stamp
    p_mixed_head->imu_num  = num;                    // number of imu data
    p_mixed_head->reserved = MAGIC_IMU_FRAME_NUM;    // magic tag

    if (0 != num) {
        update_rtp_imu_stats(&g_rtp_stats, num);
        
        clock_gettime(CLOCK_REALTIME, &p2);
        update_rtp_pack_a_time(&g_rtp_stats, timespec_diff_us(&p1, &p2));
        return buffer;
    } 

    clock_gettime(CLOCK_REALTIME, &p2);
    update_rtp_pack_a_time(&g_rtp_stats, timespec_diff_us(&p1, &p2));
    return NULL;    
}

/*
 * Procedure B: IMU + IMG data
 * Handle RTP image data, and IMU can accompanied also.
 */
char* pack_b_udp_packet(char *buffer, size_t *len, bool first_rtp) {
    struct timespec p1, p2;
    clock_gettime(CLOCK_REALTIME, &p1);

    char* p_buffer = NULL;
    uint32_t offset;

    uint32_t num = get_rb_count(&g_ring_buff);          
    if (0 != num) {
        num = (num < FORWARD_RTP_IMU_NUM)? num: FORWARD_RTP_IMU_NUM;
        p_buffer = RTP_BUFFER_ADDR(buffer) - sizeof(mix_head_t) - num*sizeof(imu_data_t);

        mix_head_t* p_mixed_head = (mix_head_t*)p_buffer;
        p_mixed_head->timestamp.img_timestamp  = GET_RTP_TIMESTAMP(buffer);
        if (first_rtp) {
            p_mixed_head->timestamp.img_sec        = previous_img_time.tv_sec;
            p_mixed_head->timestamp.img_nsec       = previous_img_time.tv_nsec; 
        } else {
            p_mixed_head->timestamp.img_sec        = lastest_img_time.tv_sec;
            p_mixed_head->timestamp.img_nsec       = lastest_img_time.tv_nsec; 
        }

        p_mixed_head->imu_num  = num;
        p_mixed_head->reserved = 0;

        //printf("%u img(%u) sec: %u nsec: %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer),  p_mixed_head->img_sec, p_mixed_head->img_nsec);

        *len   = sizeof(mix_head_t);
        offset = sizeof(mix_head_t);
        for (int i = 0; i < num; i++) {
            (void)pop_rb(&g_ring_buff, (imu_data_t*)(p_buffer+offset));

            bool is_before = is_imu_before((imu_data_t*)(p_buffer+offset), &previous_img_time);
            if (is_before) { //invalidate this outdated data
                imu_data_t* pimu = (imu_data_t*)(p_buffer+offset);
                pimu->imu_sec  = 0;
                pimu->imu_nsec = 0;
                g_rtp_stats.rtp_imu_in_img_droped++;
            } else {
                g_rtp_stats.rtp_imu_in_img_transfered++;
            }
            offset += sizeof(imu_data_t);
            *len   += sizeof(imu_data_t);
        }
    } else {
        p_buffer = RTP_BUFFER_ADDR(buffer) - sizeof(mix_head_t);
        *len = sizeof(mix_head_t);
        memset(p_buffer, 0, sizeof(mix_head_t));

        mix_head_t* p_mixed_head = (mix_head_t*)p_buffer;
        p_mixed_head->timestamp.img_timestamp  = GET_RTP_TIMESTAMP(buffer);
        if (first_rtp) {
            p_mixed_head->timestamp.img_sec        = previous_img_time.tv_sec;
            p_mixed_head->timestamp.img_nsec       = previous_img_time.tv_nsec; 
        } else {
            p_mixed_head->timestamp.img_sec        = lastest_img_time.tv_sec;
            p_mixed_head->timestamp.img_nsec       = lastest_img_time.tv_nsec; 
        }
    }
    update_rtp_imu_plus_img_stats(&g_rtp_stats, num);

    clock_gettime(CLOCK_REALTIME, &p2);
    update_rtp_pack_b_time(&g_rtp_stats, timespec_diff_us(&p1, &p2));
    return p_buffer;
}

/*
 * Procedure C: IMU data
 * It's NOT the case when RTP streaming, but might be error.
 * Not a valid RTP packet, handling IMU data if necessary
 */
char* pack_c_udp_packet(char *buffer, size_t *len) {
    struct timespec p1, p2;
    clock_gettime(CLOCK_REALTIME, &p1);

    char* p_buffer = NULL;
    uint32_t offset;

    uint32_t num = get_rb_count(&g_ring_buff);
    if (0 != num) {
        num = (num < FORWARD_RTP_IMU_NUM)? num: FORWARD_RTP_IMU_NUM;
        p_buffer = RTP_BUFFER_ADDR(buffer) - sizeof(mix_head_t) - num*sizeof(imu_data_t);

        mix_head_t* p_mixed_head = (mix_head_t*)p_buffer;
        p_mixed_head->timestamp.base_sec  = 0;
        p_mixed_head->timestamp.base_nsec = 0; 
        p_mixed_head->imu_num             = num;
        p_mixed_head->reserved            = 0;

        //printf("%u img(%u) sec: %u nsec: %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer),  p_mixed_head->img_sec, p_mixed_head->img_nsec);

        *len   = sizeof(mix_head_t);
        offset = sizeof(mix_head_t);
        for (int i = 0; i < num; i++) {
            (void)pop_rb(&g_ring_buff, (imu_data_t*)(p_buffer+offset));

            bool is_before = is_imu_before((imu_data_t*)(p_buffer+offset), &previous_img_time);
            if (is_before) { //invalidate this outdated data
                imu_data_t* pimu = (imu_data_t*)(p_buffer+offset);
                pimu->imu_sec  = 0;
                pimu->imu_nsec = 0;
            }
            offset += sizeof(imu_data_t);
            *len   += sizeof(imu_data_t);
        }
    } else {
        p_buffer = RTP_BUFFER_ADDR(buffer) - sizeof(mix_head_t);
        *len = sizeof(mix_head_t);
        memset(p_buffer, 0, sizeof(mix_head_t));
    }
    update_rtp_imu_invalid_img_stats(&g_rtp_stats, num);

    clock_gettime(CLOCK_REALTIME, &p2);
    update_rtp_pack_c_time(&g_rtp_stats, timespec_diff_us(&p1, &p2));
    return p_buffer;
}

void get_rtp_data(int local_socket, char *remote_ip, uint16_t remote_port) {
    char buffer[FORWARD_BUF_LEN];

    while (running) {
        static unsigned int packet_count = 1;
        static unsigned int packet_error = 0;
        size_t len;
        struct timespec p1, p2;

        clock_gettime(CLOCK_REALTIME, &p1);
#if 1
        fd_set fds;
        struct timeval tv;
        int retval;

        FD_ZERO(&fds);
        FD_SET(local_socket, &fds);

        tv.tv_sec  = RTP_LOCAL_TO_SEC;
        tv.tv_usec = g_uav_config.rtp_local_timeout;

        ssize_t recv_len;
        retval = select(local_socket + 1, &fds, NULL, NULL, &tv);

        if (retval == -1) {
            perror("select() error");
        } else if (retval == 0) {
            //printf("Timeout occurred after 1ms, no data received.\n");
            recv_len = -1;
        } else {
            recv_len = recvfrom(local_socket, RTP_BUFFER_ADDR(buffer), RTP_BUFFER_SIZE(buffer), 0, NULL, NULL);
            if (recv_len == -1) {
                perror("recvfrom failed");
            } else {
                //printf("Received %zd bytes of data\n", recv_len);
            }
        }
#else
        ssize_t recv_len = recvfrom(local_socket, RTP_BUFFER_ADDR(buffer), RTP_BUFFER_SIZE(buffer), 0, NULL, NULL);
#endif
        clock_gettime(CLOCK_REALTIME, &p2);
        update_rtp_recv_time(&g_rtp_stats, timespec_diff_us(&p1, &p2));

        /*
         * Procedure A: IMU data
         *
         * IDLE UPD time for IMU, handling imu sensor data
         * Please try to empty imu ring buffer
         */
        if (recv_len < 0) {
            char* p_buffer = pack_a_udp_packet(buffer, &len);
            if (p_buffer) {
                forward_udp_packets(local_socket, p_buffer, len, 'm', remote_ip, remote_port);
            }
            continue;
        }


        /*
         * UDP packet from RTP source
         * Check if the received packet is a valid RTP packet
         */
        bool valid = is_valid_rtp_packet((const uint8_t *)RTP_BUFFER_ADDR(buffer), recv_len);
        update_rtp_packet_stats(&g_rtp_stats, valid, recv_len);

        if (valid) {

            /*
            * Procedure: RTP packet stats
            * 1. statistics 
            * 2. time sync
            */
            clock_gettime(CLOCK_REALTIME, &p1);
            bool is_packet_lost;
            bool is_first_rtp = is_first_packet_of_frame(GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_MARKER(buffer), &is_packet_lost);
            if(is_first_rtp && GET_RTP_SEQUENCE_NUMBER(buffer) > previous_rtp_seqence_id){
                // update time
                previous_rtp_time.tv_sec  = lastest_rtp_time.tv_sec;
                previous_rtp_time.tv_nsec = lastest_rtp_time.tv_nsec;

                previous_img_time.tv_sec  = lastest_img_time.tv_sec;
                previous_img_time.tv_nsec = lastest_img_time.tv_nsec;

                clock_gettime(CLOCK_REALTIME, &lastest_rtp_time);
                (void)estimate_time(&g_sync_time, &lastest_img_time, GET_RTP_TIMESTAMP(buffer));
                //(void)time_minus_us(&lastest_rtp_time, g_rtp_stats.rtp_latest_delivery_per_frame);
                //(void)time_minus_us(&lastest_rtp_time, g_rtp_stats.rtp_max_delivery_per_frame);
                //(void)time_minus_us(&lastest_rtp_time, g_rtp_stats.frame_estimate_interval);

                update_rtp_head_stats(&g_rtp_stats);
                /*
                 * RTP video timestamp sync
                 */
                printf("s");
                fflush(stdout);
                unsigned int calculated_timestamp = calculate_timestamp(&g_sync_time, &lastest_rtp_time);
                if (0 == calculated_timestamp) {
                    packet_error++;
                    synchronize_time_ex(&g_sync_time, GET_RTP_TIMESTAMP(buffer), &lastest_rtp_time);
                } else {
                    static bool first_rtp_accurate_time_sync = true;
                    int64_t delta_timestamp = (int64_t)(calculated_timestamp*1.0 - (int64_t)GET_RTP_TIMESTAMP(buffer));

                    if (first_rtp_accurate_time_sync) {
                        first_rtp_accurate_time_sync = false;
                        synchronize_time_ex(&g_sync_time, previous_rtp_timestamp, &previous_rtp_time);
                        //printf("%lld %u %u\n", delta_timestamp, calculated_timestamp, GET_RTP_TIMESTAMP(buffer));
                    } else {
                        //printf("%lld %u %u\n", delta_timestamp, calculated_timestamp, GET_RTP_TIMESTAMP(buffer));
                        if (is_stamp_in_threshold(&g_sync_time, delta_timestamp)){
                            //printf("%u mon %u vs %u --> delta %lld\n", 
                            //        GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), (uint32_t)calculated_timestamp, delta_timestamp);
                        } else {
                            packet_error++;
                            synchronize_time_ex(&g_sync_time, (int64_t)GET_RTP_TIMESTAMP(buffer), &lastest_rtp_time);
                            //inc_sync_clock(&g_sync_time);
                            //printf("\033[1;31m%u sync(%u) delta(%lld) clock(%f)\033[0m\n", 
                            //        GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer), delta_timestamp, g_sync_time.clock_hz);
                        }
                    }
                }
            }else{
                //printf("body-> seq(%u) timestamp: %u\n", GET_RTP_SEQUENCE_NUMBER(buffer), GET_RTP_TIMESTAMP(buffer));
                update_rtp_body_stats(&g_rtp_stats); 
            }
            clock_gettime(CLOCK_REALTIME, &p2);
            update_rtp_sync_time(&g_rtp_stats, timespec_diff_us(&p1, &p2));

            packet_count++;
            previous_rtp_seqence_id = GET_RTP_SEQUENCE_NUMBER(buffer);
            previous_rtp_timestamp  = GET_RTP_TIMESTAMP(buffer);

            /*
            * Procedure B: IMU + IMG data
            *
            * Handle IMU data, ahead of real RTP packet
            */
            char* p_buffer = pack_b_udp_packet(buffer, &len, is_first_rtp);
            forward_udp_packets(local_socket, p_buffer, recv_len+len, 't', remote_ip, remote_port);
        } else {

            /*
            * Procedure C: IMU data
            *
            * Not a valid RTP packet, handling IMU data if necessary
            */
            char* p_buffer = pack_c_udp_packet(buffer, &len);
            forward_udp_packets(local_socket, p_buffer, recv_len+len, 'i', remote_ip, remote_port);
        }
    }

    // Print RTP statistics
    print_rtp_stats(&g_rtp_stats);
    print_rb_stats(&g_ring_buff);
}
