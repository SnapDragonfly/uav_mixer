
#include <stdio.h>
#include <limits.h>
#include <float.h>
#include <stdint.h>

#include "rtp_statistics.h"

void print_rtp_packet_histogram(const rtp_stats_t *stats) {
    uint32_t rtp_scale_factor = 1;
    uint32_t last_bucket = 0;
    for (int k = 0; k <= MAX_RTP_PACKETS; k++) {
        if (stats->packet_distribution[k] > rtp_scale_factor) {
            rtp_scale_factor = stats->packet_distribution[k];
        }
        if(stats->packet_distribution[k] > 0){
            last_bucket = k;
        }
    }
    if( rtp_scale_factor > SCALE_FACTOR) {
        rtp_scale_factor = rtp_scale_factor/SCALE_FACTOR;
    }else{
        rtp_scale_factor = 1;
    }

    printf("RTP Packet Count Histogram(scale %d):\n", rtp_scale_factor);
    
    for (int i = 0; i <= last_bucket; i++) {
        printf("%02d RTP packets: %2d | ", i, stats->packet_distribution[i]);
        
        for (int j = 0; j < stats->packet_distribution[i] / rtp_scale_factor; j++) {
            printf("*");
        }
        printf("\n");
    }
}

void print_fps_histogram(const rtp_stats_t *stats) {
    uint32_t fps_scale_factor = 1;
    uint32_t last_bucket = 0;
    for (int k = 0; k <= MAX_FRAME_PER_SECOND; k++) {
        if (stats->frame_distribution[k] > fps_scale_factor) {
            fps_scale_factor = stats->frame_distribution[k];
        }
        if(stats->frame_distribution[k] > 0){
            last_bucket = k;
        }
    }
    if( fps_scale_factor > SCALE_FACTOR) {
        fps_scale_factor = fps_scale_factor/SCALE_FACTOR;
    }else{
        fps_scale_factor = 1;
    }

    printf("Frame Per Second Histogram(scale %d):\n", fps_scale_factor);
    
    for (int i = 0; i <= last_bucket; i++) {
        printf("%03d FPS: %2d | ", i, stats->frame_distribution[i]);
        
        for (int j = 0; j < stats->frame_distribution[i] / fps_scale_factor; j++) {
            printf("*");
        }
        printf("\n");
    }
}


void init_rtp_stats(rtp_stats_t *stats) {
    stats->valid_count   = 0;
    stats->invalid_count = 0;
    stats->max_recv_len  = 0;
    stats->min_recv_len  = SSIZE_MAX;
    stats->packet_distribution_overflow = 0;

    for (int i = 0; i <= MAX_RTP_PACKETS; i++) {
        stats->packet_distribution[i] = 0;
    }

    for (int i = 0; i <= MAX_FRAME_PER_SECOND; i++) {
        stats->frame_distribution[i] = 0;
    }

    stats->rtp_last_time_of_begin_frame.tv_sec  = 0;
    stats->rtp_last_time_of_begin_frame.tv_usec = 0;

    stats->rtp_last_time_of_end_frame.tv_sec    = 0;
    stats->rtp_last_time_of_end_frame.tv_usec   = 0;

    stats->rtp_packets_count_per_frame  = 0;
    stats->rtp_max_packets_per_frame    = 0;
    stats->rtp_min_packets_per_frame    = UINT16_MAX;

    stats->rtp_max_delivery_per_frame   = 0;
    stats->rtp_min_delivery_per_frame   = DBL_MAX;

    stats->rtp_max_gap_per_frame        = 0;
    stats->rtp_min_gap_per_frame        = DBL_MAX;

    stats->frame_max_interval           = 0;
    stats->frame_min_interval           = DBL_MAX;
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

void update_rtp_recv_len(rtp_stats_t *stats, ssize_t len){
    if (len > stats->max_recv_len){
        stats->max_recv_len = len;
    }

    if (len < stats->min_recv_len){
        stats->min_recv_len = len;
    }
}

void update_rtp_head_stats(rtp_stats_t *stats){
    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    if(0 != stats->rtp_last_time_of_begin_frame.tv_sec && 0 != stats->rtp_last_time_of_begin_frame.tv_usec){
        double delivery_diff = (stats->rtp_last_time_of_end_frame.tv_sec - stats->rtp_last_time_of_begin_frame.tv_sec) * 1000000.0 
                             + (stats->rtp_last_time_of_end_frame.tv_usec - stats->rtp_last_time_of_begin_frame.tv_usec);
        double gap_diff      = (current_time.tv_sec - stats->rtp_last_time_of_end_frame.tv_sec) * 1000000.0 
                             + (current_time.tv_usec - stats->rtp_last_time_of_end_frame.tv_usec);
        double interval_diff = (current_time.tv_sec - stats->rtp_last_time_of_begin_frame.tv_sec) * 1000000.0 
                             + (current_time.tv_usec - stats->rtp_last_time_of_begin_frame.tv_usec);

        //printf("delivery_diff: %e  gap_diff: %e\n", delivery_diff, gap_diff);
        //printf("interval_diff: %e  -  %.02e Hz\n", interval_diff, 1000000/interval_diff);

        if (delivery_diff > stats->rtp_max_delivery_per_frame){
            stats->rtp_max_delivery_per_frame = delivery_diff;
        }

        if(delivery_diff < stats->rtp_min_delivery_per_frame){
            stats->rtp_min_delivery_per_frame = delivery_diff;
        }

        if(stats->rtp_packets_count_per_frame > stats->rtp_max_packets_per_frame){
            stats->rtp_max_packets_per_frame = stats->rtp_packets_count_per_frame;
        }

        if(stats->rtp_packets_count_per_frame < stats->rtp_min_packets_per_frame){
            stats->rtp_min_packets_per_frame = stats->rtp_packets_count_per_frame;
        }

        if(gap_diff > stats->rtp_max_gap_per_frame){
            stats->rtp_max_gap_per_frame = gap_diff;
        }

        if(gap_diff < stats->rtp_min_gap_per_frame){
            stats->rtp_min_gap_per_frame = gap_diff;
        }

        if(interval_diff > stats->frame_max_interval){
            stats->frame_max_interval = interval_diff;
        }

        if(interval_diff < stats->frame_min_interval){
            stats->frame_min_interval = interval_diff;
        }

        if(stats->rtp_packets_count_per_frame > MAX_RTP_PACKETS){
            stats->packet_distribution_overflow++;
            printf("packets per frame statistic overflow: %u > %u\n", stats->rtp_packets_count_per_frame, MAX_RTP_PACKETS);
        }else{
            stats->packet_distribution[stats->rtp_packets_count_per_frame]++;
        }

        uint32_t fps = 1000000/interval_diff;
        if(fps > MAX_FRAME_PER_SECOND){
            fps = MAX_FRAME_PER_SECOND;
        }
        stats->frame_distribution[fps]++;
    }

    stats->rtp_last_time_of_begin_frame.tv_sec = current_time.tv_sec;
    stats->rtp_last_time_of_begin_frame.tv_usec = current_time.tv_usec;

    stats->rtp_packets_count_per_frame = 1;
}

void update_rtp_body_stats(rtp_stats_t *stats){
    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    stats->rtp_last_time_of_end_frame.tv_sec = current_time.tv_sec;
    stats->rtp_last_time_of_end_frame.tv_usec = current_time.tv_usec;

    stats->rtp_packets_count_per_frame++;
}

void print_rtp_stats(const rtp_stats_t *stats) {
    printf("\n");
    printf("-- Summary ---------------\n");
    printf("Max frame delivery time: %e\n", stats->rtp_max_delivery_per_frame);
    printf("Min frame delivery time: %e\n", stats->rtp_min_delivery_per_frame);
    printf("     Max frame gap time: %e\n", stats->rtp_max_gap_per_frame);
    printf("     Min frame gap time: %e\n", stats->rtp_min_gap_per_frame);
    printf("Max frame interval time: %e  -  %.02e\n", stats->frame_max_interval, 1000000/stats->frame_max_interval);
    printf("Min frame interval time: %e  -  %.02e\n", stats->frame_min_interval, 1000000/stats->frame_min_interval);
    printf("--------------------------\n");
    printf("    Total max RTP packets: %u\n", stats->rtp_max_packets_per_frame);
    printf("    Total min RTP packets: %u\n", stats->rtp_min_packets_per_frame);
    printf("  Total valid RTP packets: %u\n", stats->valid_count);
    printf("Total invalid RTP packets: %u\n", stats->invalid_count);
    printf("    Max RTP packet length: %u\n", stats->max_recv_len);
    printf("    Min RTP packet length: %u\n", stats->min_recv_len);
    printf("  Max RTP packet overflow: %u\n", stats->packet_distribution_overflow);

#if 0
    printf("RTP packets distribution:\n");
    for (int i = 0; i <= MAX_RTP_PACKETS; i++) {
        printf("%02d RTP packets: %u\n", i, stats->packet_distribution[i]);
    }
#else
    print_rtp_packet_histogram(stats);
#endif
    printf("--------------------------\n");
    print_fps_histogram(stats);
    printf("--------------------------\n");
}