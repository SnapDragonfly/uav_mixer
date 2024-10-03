#include "rtp_statistics.h"
#include <stdio.h>

void init_rtp_stats(rtp_stats_t *stats) {
    stats->valid_count = 0;
    stats->invalid_count = 0;
}

bool is_valid_rtp_packet(const uint8_t *data, size_t length) {
    if (length < sizeof(rtp_header_t)) {
        return false; // Packet too short to be a valid RTP packet
    }

    const rtp_header_t *header = (const rtp_header_t *)data;

    // Check RTP version (should be 2)
    if ((header->version & 0xC0) >> 6 != 2) {
        return false; // Invalid RTP version
    }

    // Additional checks can be added here

    return true; // Valid RTP packet
}

void update_rtp_stats(rtp_stats_t *stats, int valid) {
    if (valid) {
        stats->valid_count++;
    } else {
        stats->invalid_count++;
    }
}

void print_rtp_stats(const rtp_stats_t *stats) {
    printf("Total valid RTP packets: %u\n", stats->valid_count);
    printf("Total invalid RTP packets: %u\n", stats->invalid_count);
}
