#ifndef IMU_PROCESS_H
#define IMU_PROCESS_H

#include <stdint.h>
#include <stdbool.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include "mavlink.h"
#pragma GCC diagnostic pop

#include "config.h"

// Define the structure with the updated stage type
typedef struct {
    uint8_t  sysid;             // System ID for MAVLink communication
    int      stage;             // Stage or state
    int64_t  time_offset_us;    // Time offset in microseconds
    bool     no_hr_imu;         // Flag for missing high-resolution IMU data
    bool     no_att_q;          // Flag for missing attitude quaternion data
    float    latest_alt;        // Latest altitude value
    float    gnd_alt;           // Ground altitude value
    float    latest_x;          // Latest x position
    float    start_x;           // Starting x position
    float    update_interval;   // Update interval in seconds
    float    update_rate;       // Update sensor data rate in Hz
    uint64_t last_us;           // last data time in us
    float att_q_x, att_q_y, att_q_z, att_q_w;
} mav_stats_t;

int initialize_mavlink(mav_stats_t *stats, float freq);
void process_mavlink(mav_stats_t *stats, mavlink_message_t* msg, mavlink_status_t* status, int uart_fd);

#endif