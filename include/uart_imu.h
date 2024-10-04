#ifndef UART_IMU_H
#define UART_IMU_H

#include <pthread.h>

#include "config.h"

// Function to initialize UART communication
int initialize_uart(const char* device, int baudrate);

// UART get imu sensor data
void get_imu_data(int uart_fd);

#endif // UART_IMU_H
