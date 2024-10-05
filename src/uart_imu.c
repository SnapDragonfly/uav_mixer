
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <string.h>
#include <errno.h>

#include "uart_imu.h"
#include "imu_process.h"

extern volatile sig_atomic_t running;
extern mav_stats_t g_mav_stats;

// Function to initialize UART communication
int initialize_uart(const char* device, int baudrate) {
    int uart_fd = open(device, O_RDWR | O_NOCTTY);
    if (uart_fd == -1) {
        perror("Failed to open UART device");
        return -1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);
    switch(baudrate) {
        case 9600: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break;
        case 19200: cfsetispeed(&options, B19200); cfsetospeed(&options, B19200); break;
        case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
        case 921600: cfsetispeed(&options, B921600); cfsetospeed(&options, B921600); break;
        // Add more cases as needed
        default: printf("Unsupported baud rate: %d", baudrate); return -2;
    }
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    tcsetattr(uart_fd, TCSANOW, &options);

    return uart_fd;
}

// Function to get imu sensor data
void get_imu_data(int uart_fd) {
    char buffer[UART_BUF_LEN];
    mavlink_message_t msg;
    mavlink_status_t status;

    while (running) {
        // Read data from UART
        int bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            // Parse each byte for a complete MAVLink message
            for (int i = 0; i < bytes_read; ++i) {
                if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
                    // Successfully received a complete MAVLink message
                    mavlink_process(&g_mav_stats, &msg, &status, uart_fd);
                }
            }
        } else if (bytes_read == -1) {
            perror("UART read error");
            break;
        }

        usleep(UART_BUF_SLEEP_US); // Sleep to avoid CPU hogging
    }
}
