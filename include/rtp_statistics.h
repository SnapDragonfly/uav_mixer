#ifndef RTP_STATISTICS_H
#define RTP_STATISTICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> 
#include <sys/time.h>

#include "config.h"

typedef struct {
    uint8_t cc : 4;          // CSRC count (4 bits)
    uint8_t extension : 1;   // Extension flag (1 bit)
    uint8_t padding : 1;     // Padding flag (1 bit)
    uint8_t version : 2;     // RTP version (2 bits)
    uint8_t payload_type : 7; // Payload type (7 bits)
    uint8_t marker : 1;      // Marker bit (1 bit)
    uint16_t sequence_number; // Sequence number (16 bits)
    uint32_t timestamp;      // Timestamp (32 bits)
    uint32_t ssrc;           // SSRC identifier (32 bits)
} rtp_header_t;

/*
  0                   1                   2                   3
  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |V=2|P|X|  CC   |M|     PT      |       sequence number         |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                           timestamp                           |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |           synchronization source (SSRC) identifier            |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |            contributing source (CSRC) identifiers (optional)  |
 |                             ....                              |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 |                         Payload Data                          |
 |                             ....                              |
 +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 Total CSRC bytes=CC×4
*/

// RTP Statistics structure
typedef struct {
    uint32_t valid_count;   // Count of valid RTP packets
    uint32_t invalid_count; // Count of invalid RTP packets

    int send_buffer_max;
    long send_time_max;
    long recv_time_max;

    long rtp_sync_time_max;
    long rtp_pack_a_time_max;
    long rtp_pack_b_time_max;
    long rtp_pack_c_time_max;

    uint32_t rtp_imu_count;
    uint16_t rtp_max_imu_per_frame;
    uint16_t rtp_min_imu_per_frame;

    uint32_t rtp_imu_plus_img_count;
    uint32_t rtp_imu_in_img_droped;
    uint16_t rtp_max_imu_plus_img_per_frame;
    uint16_t rtp_min_imu_plus_img_per_frame;

    uint32_t rtp_imu_invalid_img_count;
    uint16_t rtp_max_imu_invalid_img_per_frame;
    uint16_t rtp_min_imu_invalid_img_per_frame;

    ssize_t  max_recv_len;  // Max received length
    ssize_t  min_recv_len;  // Min received length
    uint32_t packet_distribution_overflow;
    uint32_t packet_distribution[MAX_RTP_PACKETS + 1];
    uint32_t frame_distribution[MAX_FRAME_PER_SECOND + 1];

    struct timeval rtp_last_time_of_begin_frame;
    struct timeval rtp_last_time_of_end_frame;

    uint16_t rtp_packets_count_per_frame;
    uint16_t rtp_max_packets_per_frame;
    uint16_t rtp_min_packets_per_frame;

    double   rtp_max_delivery_per_frame;
    double   rtp_min_delivery_per_frame;
    double   rtp_max_gap_per_frame;
    double   rtp_min_gap_per_frame;
    double   frame_max_interval;
    double   frame_min_interval;

    double   frame_estimate_interval;
    uint32_t frame_data_delay_count;
    uint32_t frame_data_ontime_count;

    uint16_t rtp_packets_previous_count;
    uint16_t rtp_packets_max_peak_bucket;
    uint16_t rtp_packets_safe_threshold;

    bool rtp_packet_interrupted;
} rtp_stats_t;

bool is_valid_rtp_packet(const uint8_t *data, size_t length);
bool is_rtp_packet_interrupted(rtp_stats_t *stats);
bool is_first_packet_of_frame(uint16_t current_seq, bool marker_bit, bool* packet_lost);
double get_rtp_packet_time_adjust(rtp_stats_t *stats);

// Function declarations
void init_rtp_stats(rtp_stats_t *stats, int fps);

void update_rtp_send_buffer_size(rtp_stats_t *stats, int size);
void update_rtp_send_time(rtp_stats_t *stats, long diff);
void update_rtp_recv_time(rtp_stats_t *stats, long diff);
void update_rtp_sync_time(rtp_stats_t *stats, long diff);
void update_rtp_pack_a_time(rtp_stats_t *stats, long diff);
void update_rtp_pack_b_time(rtp_stats_t *stats, long diff);
void update_rtp_pack_c_time(rtp_stats_t *stats, long diff);

void update_rtp_imu_stats(rtp_stats_t *stats, uint32_t num);
void update_rtp_imu_plus_img_stats(rtp_stats_t *stats, uint32_t num);
void update_rtp_imu_invalid_img_stats(rtp_stats_t *stats, uint32_t num);
void update_rtp_packet_stats(rtp_stats_t *stats, int valid, ssize_t len);
void update_rtp_interruption(rtp_stats_t *stats);
void update_rtp_head_stats(rtp_stats_t *stats);
void update_rtp_body_stats(rtp_stats_t *stats);
void print_rtp_stats(const rtp_stats_t *stats);

#endif // RTP_STATISTICS_H
