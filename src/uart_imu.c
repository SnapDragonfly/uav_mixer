
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <sys/select.h>

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

    // Flush both input and output buffers of the serial port
    if (tcflush(uart_fd, TCIOFLUSH) != 0) {
        perror("Failed to flush the serial port buffers");
        return 1;
    }

    struct termios options;
    tcgetattr(uart_fd, &options);

    options.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    options.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    options.c_cflag &= ~CSIZE; // Clear all bits that set the data size
    options.c_cflag |= CS8; // 8 bits per byte (most common)
    options.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    options.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    // Local modes - Disable canonical mode, echo, signals, etc.
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    // Input modes - Disable software flow control, and special handling of received bytes
    options.c_iflag  &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    options.c_iflag  &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special input handling

    // Output modes - Disable post-processing of output bytes
    options.c_oflag  &= ~OPOST; // Prevent special interpretation of output bytes
    options.c_oflag  &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    // Set the minimum number of characters to read before read() returns (VMIN)
    options.c_cc[VMIN]  = 1;  // Read at least 1 byte to prevent read from waiting for too much data
    options.c_cc[VTIME] = 1; // Wait 0.1 seconds for a character, then return if no more data

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

// Function to get imu sensor data (blocking mode with optional timeout)
void get_imu_data(int uart_fd) {
    char buffer[UART_BUF_LEN];
    mavlink_message_t msg;
    mavlink_status_t status;

    fd_set read_fds;
    struct timeval timeout;

    while (running) {
        // Set up the file descriptor set
        FD_ZERO(&read_fds);
        FD_SET(uart_fd, &read_fds);

        // Set timeout (2 seconds???)
        timeout.tv_sec  = UART_TO_SEC;
        timeout.tv_usec = UART_TO_USEC;

        // Wait for data to be available on the UART (blocking or with timeout)
        int ret = select(uart_fd + 1, &read_fds, NULL, NULL, &timeout);
        if (ret > 0) {
            // Check if the UART has data to read
            if (FD_ISSET(uart_fd, &read_fds)) {
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
                    continue;
                }
            }
        } else if (ret == 0) {
            // Timeout occurred, no data received
            //printf("UART read timeout\n");
        } else if (ret == -1) {
            // Error occurred in select
            if (errno == EINTR) {
                // The select was interrupted by a signal, continue the loop and try again
                continue;
            } else {
                // Handle other errors
                perror("select error");
                continue;
            }
        }
    }
}


