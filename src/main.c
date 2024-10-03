#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include "udp_forwarder.h"

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

int main() {
    // Register the signal handler for SIGINT
    signal(SIGINT, handle_sigint);

    int local_socket = initialize_udp_socket();
    printf("UDP Forwarder started. Listening on port %d and forwarding to %s:%d\n", LOCAL_PORT, FORWARD_IP, FORWARD_PORT);
    
    // Create a thread for forwarding UDP packets
    pthread_t forward_thread;
    if (pthread_create(&forward_thread, NULL, udp_forward_thread, &local_socket) != 0) {
        perror("Failed to create thread");
        close(local_socket);
        return 1; // Exit if thread creation fails
    }

    // Main loop to keep the program running until SIGINT is received
    while (running) {
        // Optionally, you can add a sleep here to reduce CPU usage
        usleep(100000); // Sleep for 100 ms
    }

    // Wait for the forwarding thread to finish
    pthread_join(forward_thread, NULL);

    close(local_socket);
    printf("UDP Forwarder stopped.\n");
    return 0;
}
