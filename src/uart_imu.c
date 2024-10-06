
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
    options.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    options.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    options.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    options.c_cflag |= CS8; // 8 bits per byte (most common)
    options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    options.c_lflag &= ~ICANON;
    options.c_lflag &= ~ECHO; // Disable echo
    options.c_lflag &= ~ECHOE; // Disable erasure
    options.c_lflag &= ~ECHONL; // Disable new-line echo
    options.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    options.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    options.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    options.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    options.c_cc[VMIN] = 0;
    switch(baudrate) {
        case 9600: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break;
        case 19200: cfsetispeed(&options, B19200); cfsetospeed(&options, B19200); break;
        case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
        case 921600: cfsetispeed(&options, B921600); cfsetospeed(&options, B921600); break;
        // Add more cases as needed
        default: printf("Unsupported baud rate: %d", baudrate); return -2;
    }
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
                    process_mavlink(&g_mav_stats, &msg, &status, uart_fd);
                }
            }
        } else if (bytes_read == -1) {
            perror("UART read error");
            break;
        }

        usleep(UART_BUF_SLEEP_US); // Sleep to avoid CPU hogging
    }
}
