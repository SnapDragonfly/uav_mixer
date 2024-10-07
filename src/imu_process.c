#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>

#include "imu_process.h"
#include "ring_buffer.h"
#include "time_sync.h"
#include "rtp_statistics.h"

extern ring_buffer_t g_ring_buff;
extern sync_time_t g_sync_time;
extern rtp_stats_t g_rtp_stats;

int initialize_mavlink(mav_stats_t *stats, float freq) {
    stats->sysid           = 0;
    stats->stage           = 0;

    stats->no_hr_imu       = true;
    stats->no_att_q        = true;

    stats->time_offset_us  = 0;

    stats->latest_alt      = 0;
    stats->gnd_alt         = 0;
    stats->latest_x        = 0;
    stats->start_x         = 0;

    stats->update_interval = 1000000/freq;
    stats->update_rate     = freq;

    stats->last_us         = 0;
    return 0;
}

void mavlink_heartbeat(mav_stats_t *stats, mavlink_message_t* msg, mavlink_status_t* status, int uart_fd) {
    int len;
    unsigned char buffer[UART_BUF_LEN];
    struct timeval tv;

    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(msg, &hb);

    if (msg->sysid != stats->sysid) {
        stats->sysid = msg->sysid;
        printf("found MAV %d\n", msg->sysid);
    }

    if (0 == stats->time_offset_us) {
        gettimeofday(&tv, NULL);
        mavlink_msg_timesync_pack(stats->sysid, MAVLINK_DEFAULT_COMP_ID, msg, 0, tv.tv_sec*1000000+tv.tv_usec, stats->sysid, 1); //fill timesync with us instead of ns
        len = mavlink_msg_to_send_buffer(&buffer[0], msg);
        write(uart_fd, buffer, len);

        mavlink_msg_system_time_pack(stats->sysid, MAVLINK_DEFAULT_COMP_ID, msg, tv.tv_sec*1000000+tv.tv_usec, 0);
        len = mavlink_msg_to_send_buffer(&buffer[0], msg);
        write(uart_fd, buffer, len);

        /*
         * Set test spot origin at(TEST_SPOT_LATITUDE, TEST_SPOT_LOGITUDE, TEST_SPOT_ALTITUDE)
         *
         * sysid: The System ID that defines which system is sending and setting the GPS origin.
         * compid: The Component ID (usually the autopilot or a specific subsystem).
         * latitude: The latitude in degrees (scaled by 1E7).
         * longitude: The longitude in degrees (scaled by 1E7).
         * altitude: The altitude above sea level in millimeters.
         * timestamp: The current time in microseconds since the UNIX epoch.
         */
        //mavlink_msg_set_gps_global_origin_pack(stats->sysid, MAVLINK_DEFAULT_COMP_ID, msg, stats->sysid, TEST_SPOT_LATITUDE, TEST_SPOT_LOGITUDE, TEST_SPOT_ALTITUDE, tv.tv_sec*1000000+tv.tv_usec);
        //len = mavlink_msg_to_send_buffer(&buffer[0], msg);
        //write(uart_fd, buffer, len);

        printf("sync... time_offset_us\n");
    }

    if (stats->no_hr_imu) {
        mavlink_msg_command_long_pack(stats->sysid, MAVLINK_DEFAULT_COMP_ID, msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_HIGHRES_IMU, stats->update_interval, 0, 0, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(&buffer[0], msg);
        write(uart_fd, buffer, len);
        printf("HIGHRES_IMU set interval %0.2fus\n", stats->update_interval);
    }

    if (stats->no_att_q) {
        mavlink_msg_command_long_pack(stats->sysid, MAVLINK_DEFAULT_COMP_ID, msg, 0, 0, MAV_CMD_SET_MESSAGE_INTERVAL, 0, MAVLINK_MSG_ID_ATTITUDE_QUATERNION, stats->update_interval, 0, 0, 0, 0, 0);
        len = mavlink_msg_to_send_buffer(&buffer[0], msg);
        write(uart_fd, buffer, len);
        printf("ATTITUDE_QUATERNION set interval %0.2fus\n", stats->update_interval);
    }

    if (hb.custom_mode == COPTER_MODE_GUIDED) {
        if (stats->stage == 0) {
            stats->stage = 1;
            gettimeofday(&tv, NULL);

            /*
             * Takeoff command when guided mode
             * 1 meter above the ground
             */
            mavlink_msg_command_long_pack(stats->sysid, MAVLINK_DEFAULT_COMP_ID, msg, stats->sysid, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 1);
            len = mavlink_msg_to_send_buffer(&buffer[0], msg);
            write(uart_fd, buffer, len);
        }
    } else {
        stats->stage = 0;
    }

    if (hb.base_mode & 128) { //armed
        if (stats->gnd_alt == 0) {
            stats->gnd_alt = stats->latest_alt;
            stats->start_x = stats->latest_x;
            printf("gnd viso alt %f, start x %f\n", stats->gnd_alt, stats->start_x);
        }
    } else { //not armed, should be on the ground
        stats->gnd_alt = 0;
    }
}

void mavlink_timesync(mav_stats_t *stats, mavlink_message_t* msg, mavlink_status_t* status, int uart_fd) {
    mavlink_timesync_t ts;
    mavlink_msg_timesync_decode(msg, &ts);
    if (ts.tc1 != 0) {
        stats->time_offset_us = ts.ts1 - ts.tc1;
        printf("sync time_offset_us = %lld\n", stats->time_offset_us);
    }
}

void mavlink_statustext(mav_stats_t *stats, mavlink_message_t* msg, mavlink_status_t* status, int uart_fd) {
    mavlink_statustext_t txt;
    mavlink_msg_statustext_decode(msg, &txt);
    printf("fc: %s\n", txt.text);
}

void mavlink_highres_imu(mav_stats_t *stats, mavlink_message_t* msg, mavlink_status_t* status, int uart_fd) {

    if(!get_sync_status(&g_sync_time)) {
        return;
    }

    mavlink_highres_imu_t hr_imu;
    mavlink_msg_highres_imu_decode(msg, &hr_imu);

#if 1
    // assign data
    if (!is_rtp_packet_interrupted(&g_rtp_stats)) {
        imu_data_t pushed_data;
#if 1 
        struct timespec tv;
        clock_gettime(CLOCK_REALTIME, &tv);
        pushed_data.imu_sec   = tv.tv_sec;           // Timestamp seconds
        pushed_data.imu_nsec  = tv.tv_nsec;  // Timestamp nanoseconds
#else 
        int64_t ts_us = hr_imu.time_usec + stats->time_offset_us;
        pushed_data.imu_sec   = ts_us / 1000000;           // Timestamp seconds
        pushed_data.imu_nsec  = (ts_us % 1000000) * 1000;  // Timestamp nanoseconds
#endif

        pushed_data.xacc      = hr_imu.xacc;               // Linear acceleration X
        pushed_data.yacc      = hr_imu.yacc;               // Linear acceleration Y
        pushed_data.zacc      = hr_imu.zacc;               // Linear acceleration Z
        pushed_data.xgyro     = hr_imu.xgyro;              // Angular velocity X
        pushed_data.ygyro     = hr_imu.ygyro;              // Angular velocity Y
        pushed_data.zgyro     = hr_imu.zgyro;              // Angular velocity Z
        pushed_data.q_w       = stats->att_q_w;            // Quaternion W
        pushed_data.q_x       = stats->att_q_x;            // Quaternion X
        pushed_data.q_y       = stats->att_q_y;            // Quaternion Y
        pushed_data.q_z       = stats->att_q_z;            // Quaternion Z

        push_rb(&g_ring_buff, &pushed_data);
    }
#else //debug
    // Print out the high-resolution IMU data
    printf("High-Resolution IMU Data:\n");
    printf("Acceleration (x, y, z): %.3f, %.3f, %.3f m/s²\n",
        hr_imu.xacc, hr_imu.yacc, hr_imu.zacc);
    printf("Angular Velocity (x, y, z): %.3f, %.3f, %.3f rad/s\n",
        hr_imu.xgyro, hr_imu.ygyro, hr_imu.zgyro);
    printf("Magnetometer (x, y, z): %.3f, %.3f, %.3f µT\n",
        hr_imu.xmag, hr_imu.ymag, hr_imu.zmag);
    printf("Pressure: %.2f hPa\n", hr_imu.abs_pressure);
    printf("Temperature: %.2f °C\n", hr_imu.temperature);
#endif

    if (stats->time_offset_us != 0 && hr_imu.time_usec > stats->last_us) {
        static int64_t effective_counts = 0;
        static int effective_rate       = 10;
        effective_counts++;
        if (0 == (effective_counts % effective_rate)){
            int64_t effective_hz = 1000000/(hr_imu.time_usec - stats->last_us);

            if (effective_hz >= 0.9*stats->update_rate && effective_hz <= 1.5*stats->update_rate){
                printf("MAVLINK_MSG_ID_HIGHRES_IMU frequency = %lldHz, should be %0.2fHz\n", 
                            effective_hz, 
                            stats->update_rate);
                
                stats->no_hr_imu = false;
                stats->no_att_q = false;
                effective_rate = stats->update_rate * 5;
            } else {
                printf("MAVLINK_MSG_ID_HIGHRES_IMU frequency = %lldHz, should be %0.2fHz\n", 
                            effective_hz, 
                            stats->update_rate);
                
                stats->no_hr_imu = true;
                stats->no_att_q = true;
                effective_rate = stats->update_rate;
            }
        }
    }
    stats->last_us = hr_imu.time_usec;
}

void mavlink_attitude_quaternion(mav_stats_t *stats, mavlink_message_t* msg, mavlink_status_t* status, int uart_fd) {

    if(!get_sync_status(&g_sync_time)) {
        return;
    }

    mavlink_attitude_quaternion_t att_q;
    mavlink_msg_attitude_quaternion_decode(msg, &att_q);

#if 1
    stats->att_q_w = att_q.q1;
    stats->att_q_x = att_q.q2;
    stats->att_q_y = att_q.q3; 
    stats->att_q_z = att_q.q4; 
#else //debug
    // Print the quaternion components
    printf("Quaternion Data:\n");
    printf("q1: %.4f, q2: %.4f, q3: %.4f, q4: %.4f\n",
           att_q.q1, att_q.q2, att_q.q3, att_q.q4);

    // Convert quaternion to roll, pitch, yaw
    float roll = atan2(2.0f * (att_q.q1 * att_q.q2 + att_q.q3 * att_q.q4), 
                        1.0f - 2.0f * (att_q.q2 * att_q.q2 + att_q.q3 * att_q.q3));

    float pitch = asin(2.0f * (att_q.q1 * att_q.q3 - att_q.q4 * att_q.q2));

    float yaw = atan2(2.0f * (att_q.q1 * att_q.q4 + att_q.q2 * att_q.q3), 
                      1.0f - 2.0f * (att_q.q3 * att_q.q3 + att_q.q4 * att_q.q4));

    // Print the converted roll, pitch, and yaw
    printf("Roll: %.2f rad, Pitch: %.2f rad, Yaw: %.2f rad\n", roll, pitch, yaw);
#endif
}

void process_mavlink(mav_stats_t *stats, mavlink_message_t* msg, mavlink_status_t* status, int uart_fd) {
    //printf("Received MAVLink message with ID %d, seq %d\n", msg->msgid, msg->seq);

    switch(msg->msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            mavlink_heartbeat(stats, msg, status, uart_fd);
            break;

        case MAVLINK_MSG_ID_TIMESYNC:
            mavlink_timesync(stats, msg, status, uart_fd);
            break;

        case MAVLINK_MSG_ID_STATUSTEXT:
            mavlink_statustext(stats, msg, status, uart_fd);
            break;

        case MAVLINK_MSG_ID_HIGHRES_IMU:
            mavlink_highres_imu(stats, msg, status, uart_fd);
            break;

        case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
            mavlink_attitude_quaternion(stats, msg, status, uart_fd);
            break;
        
        default: 
            break;
    }
}

