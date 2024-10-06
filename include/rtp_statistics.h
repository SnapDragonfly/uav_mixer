#ifndef RTP_STATISTICS_H
#define RTP_STATISTICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> 
#include <sys/time.h>

#include "config.h"

// RTP Header
typedef struct {
    uint8_t version;     // RTP version (2 bits)
    uint8_t padding;     // Padding flag (1 bit)
    uint8_t extension;   // Extension flag (1 bit)
    uint8_t cc;          // CSRC count (4 bits)
    uint8_t marker;      // Marker bit (1 bit)
    uint8_t payload_type; // Payload type (7 bits)
    uint16_t sequence_number; // Sequence number (16 bits)
    uint32_t timestamp;  // Timestamp (32 bits)
    uint32_t ssrc;       // SSRC identifier (32 bits)
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
double get_rtp_packet_time_adjust(rtp_stats_t *stats);

// Function declarations
void init_rtp_stats(rtp_stats_t *stats, int fps);
void update_rtp_packet_stats(rtp_stats_t *stats, int valid);
void update_rtp_interruption(rtp_stats_t *stats);
void update_rtp_recv_len(rtp_stats_t *stats, ssize_t len);
void update_rtp_head_stats(rtp_stats_t *stats);
void update_rtp_body_stats(rtp_stats_t *stats);
void print_rtp_stats(const rtp_stats_t *stats);

#endif // RTP_STATISTICS_H
