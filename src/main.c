#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>

#include "udp_forwarder.h"
#include "uart_imu.h"

// Global variable to track if the program should continue running
volatile sig_atomic_t running = 1;

// Signal handler for SIGINT
void handle_sigint(int signo) {
    running = 0; // Set running flag to 0
}

// Thread function for forwarding UDP packets
void* udp_forward_thread(void* arg) {
    int local_socket = *(int*)arg; // Get the socket from the argument
    forward_udp_packets(local_socket);
    return NULL;
}

// UART thread function
void* uart_imu_thread(void* arg) {
    int uart_fd = *((int*)arg);
    get_imu_data(uart_fd);
    return NULL;
}

int main() {
    // Register the signal handler for SIGINT (CTRL+C)
    signal(SIGINT, handle_sigint);

    // Initialize UDP socket
    int local_socket = initialize_udp_socket();
    printf("UDP Forwarder started. Listening on port %d and forwarding to %s:%d\n", LOCAL_PORT, FORWARD_IP, FORWARD_PORT);
    
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
        // Optionally, you can add a sleep here to reduce CPU usage
        usleep(1000000); // Sleep for 1000 ms
    }

    // Wait for the forwarding and UART threads to finish
    pthread_join(forward_thread, NULL);
    pthread_join(uart_thread, NULL);

    close(local_socket);
    printf("UDP Forwarder and UART communication stopped.\n");
    return 0;
}
