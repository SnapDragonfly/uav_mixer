#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <getopt.h>
#include <sys/resource.h>

#include "udp_forwarder.h"
#include "uart_imu.h"
#include "imu_process.h"
#include "rtp_statistics.h"
#include "time_sync.h"
#include "ring_buffer.h"
#include "version.h"

// Global variable to track if the program should continue running
volatile sig_atomic_t running = 1;
rtp_stats_t g_rtp_stats;
sync_time_t g_sync_time;
mav_stats_t g_mav_stats;
ring_buffer_t g_ring_buff;
uav_config_t g_uav_config;

// Function to print help message
void print_help() {
    print_version();
    printf("Usage: config_parser [options]\n");
    printf("Options:\n");
    printf("  -f, --mavlink-freq <freq>         Set MAVLink frequency (default: %d)\n", MAVLINK_DEFAULT_FREQ);
    printf("  -r, --rtp-fps <fps>               Set RTP frames per second (default: %d)\n", RTP_FPS_RATE);
    printf("  -c, --rtp-clock-freq <freq>       Set RTP clock frequency in Hz (default: %d)\n", RTP_CLOCK_FREQ_HZ);
    printf("  -p, --rtp-local-port <port>       Set RTP local port (default: %d)\n", RTP_LOCAL_PORT);
    printf("  -t, --rtp-local-timeout <timeout> Set RTP local timeout (default: %d us)\n", RTP_LOCAL_TO_USEC);
    printf("  -i, --forward-ip <ip>             Set forward destination IP (default: %s)\n", FORWARD_IP);
    printf("  -P, --forward-port <port>         Set forward destination port (default: %d)\n", FORWARD_PORT);
    printf("  -d, --uart-device <device>        Set UART device (default: %s)\n", UART_DEVICE);
    printf("  -b, --uart-baudrate <baudrate>    Set UART baud rate (default: %d)\n", UART_BAUDRATE);
    printf("  -h, --help                        Show this help message\n");
}

void parse_args(int argc, char *argv[], uav_config_t *config) {
    // Set default values
    config->mavlink_freq = MAVLINK_DEFAULT_FREQ;
    config->rtp_fps = RTP_FPS_RATE;
    config->rtp_clock_freq_hz = RTP_CLOCK_FREQ_HZ;
    config->rtp_local_port = RTP_LOCAL_PORT;
    config->rtp_local_timeout = RTP_LOCAL_TO_USEC;
    strncpy(config->forward_ip, FORWARD_IP, sizeof(config->forward_ip));
    config->forward_port = FORWARD_PORT;
    strncpy(config->uart_device, UART_DEVICE, sizeof(config->uart_device));
    config->uart_baudrate = UART_BAUDRATE;

    // Initialize option variables
    int opt;
    struct option long_options[] = {
        {"mavlink-freq", required_argument, NULL, 'f'},
        {"rtp-fps", required_argument, NULL, 'r'},
        {"rtp-clock-freq", required_argument, NULL, 'c'},
        {"rtp-local-port", required_argument, NULL, 'p'},
        {"rtp-local-timeout", required_argument, NULL, 't'},
        {"forward-ip", required_argument, NULL, 'i'},
        {"forward-port", required_argument, NULL, 'P'},
        {"uart-device", required_argument, NULL, 'd'},
        {"uart-baudrate", required_argument, NULL, 'b'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    // Parse command-line options
    while ((opt = getopt_long(argc, argv, "f:r:c:p:t:i:P:d:b:h", long_options, NULL)) != -1) {
        switch (opt) {
            case 'f':
                config->mavlink_freq = atoi(optarg);
                break;
            case 'r':
                config->rtp_fps = atoi(optarg);
                break;
            case 'c':
                config->rtp_clock_freq_hz = atoi(optarg);
                break;
            case 'p':
                config->rtp_local_port = atoi(optarg);
                break;
            case 't':
                config->rtp_local_timeout = atoi(optarg);
                break;
            case 'i':
                strncpy(config->forward_ip, optarg, sizeof(config->forward_ip));
                break;
            case 'P':
                config->forward_port = atoi(optarg);
                break;
            case 'd':
                strncpy(config->uart_device, optarg, sizeof(config->uart_device));
                break;
            case 'b':
                config->uart_baudrate = atoi(optarg);
                break;
            case 'h':
            default: // Show help message
                print_help();
                exit(0);
        }
    }
}

// Signal handler for SIGINT
void handle_sig_int_term(int signo) {
    running = 0; // Set running flag to 0
}

// Thread function for forwarding UDP packets
void* udp_forward_thread(void* arg) {
    pthread_t thread = pthread_self();  // Get the current thread ID
    int policy;
    struct sched_param param;

#if 0
    // Set the thread's scheduling policy and priority
    param.sched_priority = 90;  // Set the priority
    if (pthread_setschedparam(thread, SCHED_FIFO, &param) != 0) {
        perror("Failed to set thread scheduling parameters");
        return NULL;
    }
#endif
    // Get the current thread's scheduling policy and priority
    if (pthread_getschedparam(thread, &policy, &param) != 0) {
        perror("Failed to get thread scheduling parameters");
        return NULL;
    }

    // Print the current thread's scheduling policy and priority
    printf("udp_forward_thread policy: %s\n", 
           (policy == SCHED_FIFO) ? "SCHED_FIFO" :
           (policy == SCHED_RR) ? "SCHED_RR" :
           (policy == SCHED_OTHER) ? "SCHED_OTHER" : "UNKNOWN");
    printf("udp_forward_thread priority: %d\n", param.sched_priority);

    int local_socket = *(int*)arg; // Get the socket from the argument
    get_rtp_data(local_socket, g_uav_config.forward_ip, g_uav_config.forward_port);

    close(local_socket);
    return NULL;
}

// UART thread function for getting imu sensor data
void* uart_imu_thread(void* arg) {
    pthread_t thread = pthread_self();  // Get the current thread ID
    int policy;
    struct sched_param param;

#if 0
    // Set the thread's scheduling policy and priority
    param.sched_priority = 10;  // Set the priority
    if (pthread_setschedparam(thread, SCHED_FIFO, &param) != 0) {
        perror("Failed to set thread scheduling parameters");
        return NULL;
    }
#endif
    // Get the current thread's scheduling policy and priority
    if (pthread_getschedparam(thread, &policy, &param) != 0) {
        perror("Failed to get thread scheduling parameters");
        return NULL;
    }

    // Print the current thread's scheduling policy and priority
    printf("uart_imu_thread policy: %s\n", 
           (policy == SCHED_FIFO) ? "SCHED_FIFO" :
           (policy == SCHED_RR) ? "SCHED_RR" :
           (policy == SCHED_OTHER) ? "SCHED_OTHER" : "UNKNOWN");
    printf("uart_imu_thread priority: %d\n", param.sched_priority);

    int uart_fd = *((int*)arg);
    get_imu_data(uart_fd);

    close(uart_fd);
    return NULL;
}

int main(int argc, char *argv[]) {
    parse_args(argc, argv, &g_uav_config);

    // Output the parsed results
    printf("MAVLINK_DEFAULT_FREQ: %d\n", g_uav_config.mavlink_freq);
    printf("RTP_FPS_RATE: %d\n", g_uav_config.rtp_fps);
    printf("RTP_CLOCK_FREQ_HZ: %d\n", g_uav_config.rtp_clock_freq_hz);
    printf("RTP_LOCAL_PORT: %d\n", g_uav_config.rtp_local_port);
    printf("FORWARD_IP: %s\n", g_uav_config.forward_ip);
    printf("FORWARD_PORT: %d\n", g_uav_config.forward_port);
    printf("UART_DEVICE: %s\n", g_uav_config.uart_device);
    printf("UART_BAUDRATE: %d\n", g_uav_config.uart_baudrate);

    // Register the signal handler for SIGINT (CTRL+C)
    signal(SIGINT, handle_sig_int_term);
    signal(SIGTERM, handle_sig_int_term);

    if (FORWARD_RTP_IMU_SIZE != sizeof(imu_data_t)) {
        perror("Fatal error: imu_data_t, check hardware/gcc compatibility!\n");
        return 1;
    }

    if (FORWARD_RTP_IMG_SIZE != sizeof(mix_head_t)) {
        perror("Fatal error: mix_head_t, check hardware/gcc compatibility!\n");
        return 1;
    }

    (void)initialize_mavlink(&g_mav_stats, g_uav_config.mavlink_freq);     // Initialize MAVLink handler
    (void)init_rtp_stats(&g_rtp_stats, g_uav_config.rtp_fps);              // Initialize RTP statistics
    (void)init_sync_system(&g_sync_time, g_uav_config.rtp_clock_freq_hz, g_uav_config.rtp_fps);  // Initialize with a clock frequency
    (void)init_rb(&g_ring_buff);


    // Initialize UDP socket
    int local_socket = initialize_udp_socket(g_uav_config.rtp_local_port);
    printf("UDP Forwarder started. Listening on port %d timeout %d and forwarding to %s:%d\n", 
            g_uav_config.rtp_local_port, g_uav_config.rtp_local_timeout, g_uav_config.forward_ip, g_uav_config.forward_port);
    
    // Create a thread for forwarding UDP packets
    pthread_t forward_thread;
    if (pthread_create(&forward_thread, NULL, udp_forward_thread, &local_socket) != 0) {
        perror("Failed to create UDP forward thread");
        close(local_socket);
        return 1;
    }

    // Initialize UART communication
    int uart_fd = initialize_uart(g_uav_config.uart_device, g_uav_config.uart_baudrate);
    if (uart_fd == -1) {
        close(local_socket);
        return 1; // Exit if UART initialization fails
    }

    // Create a thread for UART IMU communication
    pthread_t uart_thread;
    if (pthread_create(&uart_thread, NULL, uart_imu_thread, &uart_fd) != 0) {
        perror("Failed to create UART thread");
        close(local_socket);
        close(uart_fd);
        return 1;
    }

    // Main loop to keep the program running until SIGINT is received
    while (running) {
        update_rtp_interruption(&g_rtp_stats);
        // Optionally, you can add a sleep here to reduce CPU usage
        sleep(1); // Sleep for 1 second
    }

    // Wait for the forwarding and UART threads to finish
    pthread_join(forward_thread, NULL);
    pthread_join(uart_thread, NULL);

    printf("UDP Forwarder and UART communication stopped.\n");
    print_version();
    
    return 0;
}
