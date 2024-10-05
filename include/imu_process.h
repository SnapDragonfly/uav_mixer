#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink.h"
#pragma GCC diagnostic pop

#include "config.h"
int initialize_mavlink(float freq);
void mavlink_process(mavlink_message_t* msg, mavlink_status_t* status, int uart_fd);

#endif