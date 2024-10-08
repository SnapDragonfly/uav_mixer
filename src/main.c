#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>

#include "udp_forwarder.h"
#include "uart_imu.h"
#include "imu_process.h"
#include "rtp_statistics.h"
#include "time_sync.h"
#include "ring_buffer.h"

// Global variable to track if the program should continue running
volatile sig_atomic_t running = 1;
rtp_stats_t g_rtp_stats;
sync_time_t g_sync_time;
mav_stats_t g_mav_stats;
ring_buffer_t g_ring_buff;

// Signal handler for SIGINT
void handle_sigint(int signo) {
    running = 0; // Set running flag to 0
}

// Thread function for forwarding UDP packets
void* udp_forward_thread(void* arg) {
    int local_socket = *(int*)arg; // Get the socket from the argument
    forward_udp_packets(local_socket, FORWARD_IP, FORWARD_PORT);
    close(local_socket);
    return NULL;
}

// UART thread function for getting imu sensor data
void* uart_imu_thread(void* arg) {
    int uart_fd = *((int*)arg);
    get_imu_data(uart_fd);
    close(uart_fd);
    return NULL;
}

int main() {
    // Register the signal handler for SIGINT (CTRL+C)
    signal(SIGINT, handle_sigint);

    if (FORWARD_RTP_IMU_LEN != sizeof(imu_data_t)) {
        perror("Fatal error: imu_data_t, check hardware/gcc compatibility!\n");
        return 1;
    }

    if (FORWARD_RTP_IMG_LEN != sizeof(mix_head_t)) {
        perror("Fatal error: mix_head_t, check hardware/gcc compatibility!\n");
        return 1;
    }


    (void)initialize_mavlink(&g_mav_stats, MAVLINK_DEFAULT_FREQ);  //Initialize MAVLink handler
    (void)init_rtp_stats(&g_rtp_stats, RTP_FPS_RATE);              // Initialize RTP statistics
    (void)init_sync_system(&g_sync_time, RTP_CLOCK_FREQ_HZ);       // Initialize with a clock frequency
    (void)init_rb(&g_ring_buff);

    // Initialize UDP socket
    int local_socket = initialize_udp_socket(RTP_LOCAL_PORT);
    printf("UDP Forwarder started. Listening on port %d and forwarding to %s:%d\n", RTP_LOCAL_PORT, FORWARD_IP, FORWARD_PORT);
    
    // Create a thread for forwarding UDP packets
    pthread_t forward_thread;
    if (pthread_create(&forward_thread, NULL, udp_forward_thread, &local_socket) != 0) {
        perror("Failed to create UDP forward thread");
        close(local_socket);
        return 1;
    }

    // Initialize UART communication
    int uart_fd = initialize_uart(UART_DEVICE, UART_BAUDRATE);
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
        sleep(1); // Sleep for 1s
    }

    // Wait for the forwarding and UART threads to finish
    pthread_join(forward_thread, NULL);
    pthread_join(uart_thread, NULL);

    printf("UDP Forwarder and UART communication stopped.\n");
    return 0;
}
