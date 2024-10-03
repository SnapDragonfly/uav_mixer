#ifndef RTP_STATISTICS_H
#define RTP_STATISTICS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h> 

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

// RTP Statistics structure
typedef struct {
    uint32_t valid_count;   // Count of valid RTP packets
    uint32_t invalid_count; // Count of invalid RTP packets
} rtp_stats_t;

// Function declarations
void init_rtp_stats(rtp_stats_t *stats);
bool is_valid_rtp_packet(const uint8_t *data, size_t length);
void update_rtp_stats(rtp_stats_t *stats, int valid);
void print_rtp_stats(const rtp_stats_t *stats);

#endif // RTP_STATISTICS_H
