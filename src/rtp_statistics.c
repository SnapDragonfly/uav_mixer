
#include <stdio.h>
#include <limits.h>
#include <float.h>
#include <stdint.h>
#include <math.h>

#include "rtp_statistics.h"

void print_rtp_packet_histogram(const rtp_stats_t *stats) {
    uint32_t rtp_scale_factor = 1;
    uint32_t last_bucket = 0;
    uint32_t total_packets = 0;    // Total number of packets
    double sum_packets = 0;         // Sum of packet counts (for calculating average)
    double sum_packets_square = 0;  // Sum of packet counts squared (for calculating RMS)

    // Calculate the maximum occurrence (for scaling) and the last non-zero bucket
    for (int k = 0; k <= MAX_RTP_PACKETS; k++) {
        if (stats->packet_distribution[k] > rtp_scale_factor) {
            rtp_scale_factor = stats->packet_distribution[k];
        }
        if (stats->packet_distribution[k] > 0) {
            last_bucket = k;
        }

        // Calculate total packets, sum of packets, and sum of packets squared
        total_packets += stats->packet_distribution[k];
        sum_packets += k * stats->packet_distribution[k];
        sum_packets_square += (k * k) * stats->packet_distribution[k];
    }

    if(0 == total_packets) {
        return;
    }

    // Calculate average and RMS
    double mean_packets = total_packets > 0 ? sum_packets / total_packets : 0;
    double rms_packets = total_packets > 0 ? sqrt(sum_packets_square / total_packets) : 0;

    if (rtp_scale_factor > SCALE_FACTOR) {
        rtp_scale_factor = rtp_scale_factor / SCALE_FACTOR;
    } else {
        rtp_scale_factor = 1;
    }

    // Print RTP packet histogram
    printf("RTP Packet Count Histogram(scale %d):\n", rtp_scale_factor);
    
    for (int i = 0; i <= last_bucket; i++) {
        printf("%02d RTP packets: %2d | ", i, stats->packet_distribution[i]);
        
        for (int j = 0; j < stats->packet_distribution[i] / rtp_scale_factor; j++) {
            printf("*");
        }
        printf("\n");
    }

    // Print average and RMS
    printf("Average RTP packets: %.2f\n", mean_packets);
    printf("    RMS RTP packets: %.2f\n", rms_packets);
}

void print_packets_delivery_histogram(const rtp_stats_t *stats) {
    uint32_t last_bucket = 0;
    uint32_t delivery_scale_factor = 1;

    // Calculate the maximum occurrence (used to scale the number of stars) and the last non-zero bucket
    for (int k = 0; k <= MAX_RTP_PACKETS; k++) {
        if (stats->rtp_packet_delivery[k] > 0) {
            last_bucket = k;
        }
    }

    if (stats->rtp_max_delivery_per_frame > SCALE_FACTOR) {
        delivery_scale_factor = stats->rtp_max_delivery_per_frame / SCALE_FACTOR;
    } else {
        delivery_scale_factor = 1;
    }

    // Print packet delivery histogram
    printf("Frame Packet Delivery Histogram\n");
    printf("Max frame delivery time: %e us\n", stats->rtp_max_delivery_per_frame);
    printf("Min frame delivery time: %e us\n", stats->rtp_min_delivery_per_frame);
    for (int i = 0; i <= last_bucket; i++) {
        printf("%03d Packet Delivery: %.0f | ", i, stats->rtp_packet_delivery[i]);
        
        for (int j = 0; j < stats->rtp_packet_delivery[i] / delivery_scale_factor; j++) {
            printf("*");
        }
        printf("\n");
    }
}

void print_packets_gap_histogram(const rtp_stats_t *stats) {
    uint32_t last_bucket = 0;
    uint32_t delivery_scale_factor = 1;

    // Calculate the maximum occurrence (used to scale the number of stars) and the last non-zero bucket
    for (int k = 0; k <= MAX_RTP_PACKETS; k++) {
        if (stats->rtp_packet_gap[k] > 0) {
            last_bucket = k;
        }
    }

    if (stats->rtp_max_gap_per_frame > SCALE_FACTOR) {
        delivery_scale_factor = stats->rtp_max_gap_per_frame / SCALE_FACTOR;
    } else {
        delivery_scale_factor = 1;
    }

    // Print packet delivery histogram
    printf("Frame Packet Gap Histogram\n");
    printf("     Max frame gap time: %e us\n", stats->rtp_max_gap_per_frame);
    printf("     Min frame gap time: %e us\n", stats->rtp_min_gap_per_frame);
    for (int i = 0; i <= last_bucket; i++) {
        printf("%03d Packet Gap: %.0f | ", i, stats->rtp_packet_gap[i]);
        
        for (int j = 0; j < stats->rtp_packet_gap[i] / delivery_scale_factor; j++) {
            printf("*");
        }
        printf("\n");
    }
}

// Calculate the standard deviation and average for the FPS histogram
void print_fps_histogram(const rtp_stats_t *stats) {
    uint32_t fps_scale_factor = 1;
    uint32_t last_bucket = 0;
    uint32_t total_frames = 0;  // Total number of frames
    double sum_fps = 0;         // Sum of FPS (for calculating the average)
    double sum_fps_square = 0;  // Sum of FPS squared (for calculating RMS)

    // Calculate the maximum occurrence (used to scale the number of stars) and the last non-zero bucket
    for (int k = 0; k <= MAX_FRAME_PER_SECOND; k++) {
        if (stats->frame_distribution[k] > fps_scale_factor) {
            fps_scale_factor = stats->frame_distribution[k];
        }
        if (stats->frame_distribution[k] > 0) {
            last_bucket = k;
        }

        // Calculate the total number of frames, the sum of FPS, and the sum of FPS squared
        total_frames += stats->frame_distribution[k];
        sum_fps += k * stats->frame_distribution[k];
        sum_fps_square += (k * k) * stats->frame_distribution[k];
    }

    if( 0 == total_frames) {
        return;
    }

    // Calculate the average and RMS
    double mean_fps = total_frames > 0 ? sum_fps / total_frames : 0;
    double rms_fps = total_frames > 0 ? sqrt(sum_fps_square / total_frames) : 0;

    if (fps_scale_factor > SCALE_FACTOR) {
        fps_scale_factor = fps_scale_factor / SCALE_FACTOR;
    } else {
        fps_scale_factor = 1;
    }

    // Print FPS histogram
    printf("Frame Per Second Histogram(scale %d):\n", fps_scale_factor);
    
    for (int i = 0; i <= last_bucket; i++) {
        printf("%03d FPS: %2d | ", i, stats->frame_distribution[i]);
        
        for (int j = 0; j < stats->frame_distribution[i] / fps_scale_factor; j++) {
            printf("*");
        }
        printf("\n");
    }

    // Print the average and RMS
    printf("Average FPS: %.2f\n", mean_fps);
    printf("    RMS FPS: %.2f\n", rms_fps);
}

void init_rtp_stats(rtp_stats_t *stats, int fps) {
    stats->valid_count   = 0;
    stats->invalid_count = 0;

    stats->send_buffer_max = 0;
    stats->send_time_max   = 0;
    stats->recv_time_max   = 0;

    stats->rtp_sync_time_max     = 0;
    stats->rtp_pack_a_time_max   = 0;
    stats->rtp_pack_b_time_max   = 0;
    stats->rtp_pack_c_time_max   = 0;

    stats->max_recv_len  = 0;
    stats->min_recv_len  = SSIZE_MAX;
    stats->packet_distribution_overflow = 0;

    for (int i = 0; i <= MAX_RTP_PACKETS; i++) {
        stats->packet_distribution[i] = 0;
    }

    stats->rtp_latest_delivery_per_frame = 0;
    for (int i = 0; i <= MAX_RTP_PACKETS; i++) {
        stats->rtp_packet_delivery[i] = 0;
    }

    stats->rtp_latest_gap_per_frame = 0;
    for (int i = 0; i <= MAX_RTP_PACKETS; i++) {
        stats->rtp_packet_gap[i] = 0;
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

    stats->frame_estimate_interval      = 1000000/fps;

    stats->frame_data_delay_count       = 0;
    stats->frame_data_ontime_count      = 0;

    stats->rtp_packets_previous_count   = 0;
    stats->rtp_packets_max_peak_bucket  = 0;
    stats->rtp_packets_safe_threshold   = 0;

    stats->rtp_imu_count                = 0;
    stats->rtp_max_imu_per_frame        = 0;
    stats->rtp_min_imu_per_frame        = UINT16_MAX;

    stats->rtp_imu_plus_img_count                = 0;
    stats->rtp_imu_in_img_droped                 = 0;
    stats->rtp_imu_in_img_transfered             = 0;
    stats->rtp_max_imu_plus_img_per_frame        = 0;
    stats->rtp_min_imu_plus_img_per_frame        = UINT16_MAX;

    stats->rtp_imu_invalid_img_count              = 0;
    stats->rtp_max_imu_invalid_img_per_frame      = 0;
    stats->rtp_min_imu_invalid_img_per_frame      = UINT16_MAX;


    stats->rtp_packet_interrupted       = true;
}

void update_rtp_send_buffer_size(rtp_stats_t *stats, int size) {
    if (size > stats->send_buffer_max) {
        stats->send_buffer_max = size;
    }
}

void update_rtp_send_time(rtp_stats_t *stats, long diff) {
    if (diff > stats->send_time_max) {
        stats->send_time_max = diff;
    }
}

void update_rtp_recv_time(rtp_stats_t *stats, long diff) {
    if (diff > stats->recv_time_max) {
        stats->recv_time_max = diff;
    }
}

void update_rtp_sync_time(rtp_stats_t *stats, long diff) {
    if (diff > stats->rtp_sync_time_max) {
        stats->rtp_sync_time_max = diff;
    }
}

void update_rtp_pack_a_time(rtp_stats_t *stats, long diff) {
    if (diff > stats->rtp_pack_a_time_max) {
        stats->rtp_pack_a_time_max = diff;
    }
}

void update_rtp_pack_b_time(rtp_stats_t *stats, long diff) {
    if (diff > stats->rtp_pack_b_time_max) {
        stats->rtp_pack_b_time_max = diff;
    }
}

void update_rtp_pack_c_time(rtp_stats_t *stats, long diff) {
    if (diff > stats->rtp_pack_c_time_max) {
        stats->rtp_pack_c_time_max = diff;
    }
}

bool is_valid_rtp_packet(const uint8_t *data, size_t length) {
    if (length < sizeof(rtp_header_t)) {
        return false; // Packet too short to be a valid RTP packet
    }

    const rtp_header_t *header = (const rtp_header_t *)data;

    // Check RTP version (should be 2)
    if ((header->version & 0x03) != 2) { // Only the last 2 bits for version
        return false; // Invalid RTP version
    }

    // Check for valid payload type (0-127 are standard types)
    if (header->payload_type > 127) {
        return false; // Invalid payload type
    }

    // Check if padding is set
    if (header->padding) {
        // Ensure there are enough bytes for the RTP header + padding length
        if (length < sizeof(rtp_header_t)) {
            return false; // Not enough data for the header
        }
        // The padding length is determined by the last byte of the packet
        uint8_t padding_length = data[length - 1]; // Last byte for padding length
        if (length < sizeof(rtp_header_t) + padding_length) {
            return false; // Not enough data for padding
        }
    }

    // Check the CSRC count and ensure it does not exceed the packet length
    size_t csrc_count = header->cc;
    if (length < sizeof(rtp_header_t) + (csrc_count * sizeof(uint32_t))) {
        return false; // Packet too short for CSRC identifiers
    }

    // Additional checks can be added here (e.g., for sequence number, timestamp)

    return true; // Valid RTP packet
}

bool is_rtp_packet_interrupted(rtp_stats_t *stats) {
    return stats->rtp_packet_interrupted;
}

double get_rtp_packet_time_adjust(rtp_stats_t *stats) {
    if (stats->rtp_packets_previous_count < stats->rtp_packets_safe_threshold) {
        return stats->rtp_min_delivery_per_frame;
    }

    return stats->rtp_max_delivery_per_frame;
}

void update_rtp_imu_stats(rtp_stats_t *stats, uint32_t num) {
    stats->rtp_imu_count++;
    if (num > stats->rtp_max_imu_per_frame) {
        stats->rtp_max_imu_per_frame = num;
    }
    if (num < stats->rtp_min_imu_per_frame) {
        stats->rtp_min_imu_per_frame = num;
    }
}

void update_rtp_imu_plus_img_stats(rtp_stats_t *stats, uint32_t num) {
    stats->rtp_imu_plus_img_count++;
    if (num > stats->rtp_max_imu_plus_img_per_frame) {
        stats->rtp_max_imu_plus_img_per_frame = num;
    }
    if (num < stats->rtp_min_imu_plus_img_per_frame) {
        stats->rtp_min_imu_plus_img_per_frame = num;
    }
}

void update_rtp_imu_invalid_img_stats(rtp_stats_t *stats, uint32_t num) {
    stats->rtp_imu_invalid_img_count++;
    if (num > stats->rtp_max_imu_invalid_img_per_frame) {
        stats->rtp_max_imu_invalid_img_per_frame = num;
    }
    if (num < stats->rtp_min_imu_invalid_img_per_frame) {
        stats->rtp_min_imu_invalid_img_per_frame = num;
    }
}

void update_rtp_interruption(rtp_stats_t *stats) {
    /*
     * Be careful, as it's NOT thread safe
     * just change status variable, which is NOT that time critical
     */

    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    /*
     * Update interruption
     */
    double diff = (current_time.tv_sec - stats->rtp_last_time_of_begin_frame.tv_sec) * 1000000.0 
                + (current_time.tv_usec - stats->rtp_last_time_of_begin_frame.tv_usec);

    if(diff > stats->frame_estimate_interval) {
        stats->rtp_packet_interrupted = true;
    } else {
        stats->rtp_packet_interrupted = false;
    }

    /*
     *
     */
    uint32_t peak_count = 0;
    uint16_t max_bucket = 0;
    for (int i = 0; i <= MAX_RTP_PACKETS; i++) {
        if(stats->packet_distribution[i] > peak_count) {
            peak_count = stats->packet_distribution[i];
            stats->rtp_packets_max_peak_bucket  =i;
        }

        if(stats->packet_distribution[i] > 0) {
            max_bucket = i;
        }
    }

    stats->rtp_packets_safe_threshold   = (max_bucket + stats->rtp_packets_max_peak_bucket)/2;
}

void update_rtp_packet_stats(rtp_stats_t *stats, int valid, ssize_t len) {
    if (valid) {
        stats->valid_count++;

        if (len > stats->max_recv_len){
            stats->max_recv_len = len;
        }

        if (len < stats->min_recv_len){
            stats->min_recv_len = len;
        }
    } else {
        stats->invalid_count++;
    }
}

void update_rtp_head_stats(rtp_stats_t *stats){
    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    if(0 != stats->rtp_last_time_of_begin_frame.tv_sec && 0 != stats->rtp_last_time_of_begin_frame.tv_usec){
        stats->rtp_latest_delivery_per_frame = (stats->rtp_last_time_of_end_frame.tv_sec - stats->rtp_last_time_of_begin_frame.tv_sec) * 1000000.0 
                             + (stats->rtp_last_time_of_end_frame.tv_usec - stats->rtp_last_time_of_begin_frame.tv_usec);
        stats->rtp_latest_gap_per_frame      = (current_time.tv_sec - stats->rtp_last_time_of_end_frame.tv_sec) * 1000000.0 
                             + (current_time.tv_usec - stats->rtp_last_time_of_end_frame.tv_usec);
        double interval_diff = (current_time.tv_sec - stats->rtp_last_time_of_begin_frame.tv_sec) * 1000000.0 
                             + (current_time.tv_usec - stats->rtp_last_time_of_begin_frame.tv_usec);

        //printf("delivery_diff: %e  gap_diff: %e\n", stats->rtp_latest_delivery_per_frame, gap_diff);
        //printf("interval_diff: %e  -  %.02e Hz\n", stats->rtp_latest_delivery_per_frame, 1000000/stats->rtp_latest_delivery_per_frame);
        if (interval_diff > stats->frame_estimate_interval){
            stats->frame_data_delay_count++;
            //printf("frame data severely delayed: %e > %e\n", stats->rtp_latest_delivery_per_frame, stats->frame_estimate_interval);
        } else {
            stats->frame_data_ontime_count++;
        }

        if (0 == stats->rtp_packet_delivery[stats->rtp_packets_count_per_frame]) {
            stats->rtp_packet_delivery[stats->rtp_packets_count_per_frame] = stats->rtp_latest_delivery_per_frame;
        } else {
            stats->rtp_packet_delivery[stats->rtp_packets_count_per_frame] = (stats->rtp_latest_delivery_per_frame + stats->rtp_packet_delivery[stats->rtp_packets_count_per_frame]) /2;
        }

        if (0 == stats->rtp_packet_gap[stats->rtp_packets_count_per_frame]) {
            stats->rtp_packet_gap[stats->rtp_packets_count_per_frame] = stats->rtp_latest_gap_per_frame;
        } else {
            stats->rtp_packet_gap[stats->rtp_packets_count_per_frame] = (stats->rtp_latest_gap_per_frame + stats->rtp_packet_gap[stats->rtp_packets_count_per_frame]) /2;
        }

        if (stats->rtp_latest_delivery_per_frame > stats->rtp_max_delivery_per_frame){
            stats->rtp_max_delivery_per_frame = stats->rtp_latest_delivery_per_frame;
        }

        if(stats->rtp_latest_delivery_per_frame < stats->rtp_min_delivery_per_frame){
            stats->rtp_min_delivery_per_frame = stats->rtp_latest_delivery_per_frame;
        }

        if(stats->rtp_packets_count_per_frame > stats->rtp_max_packets_per_frame){
            stats->rtp_max_packets_per_frame = stats->rtp_packets_count_per_frame;
        }

        if(stats->rtp_packets_count_per_frame < stats->rtp_min_packets_per_frame){
            stats->rtp_min_packets_per_frame = stats->rtp_packets_count_per_frame;
        }

        if(stats->rtp_latest_gap_per_frame > stats->rtp_max_gap_per_frame){
            stats->rtp_max_gap_per_frame = stats->rtp_latest_gap_per_frame;
        }

        if(stats->rtp_latest_gap_per_frame < stats->rtp_min_gap_per_frame){
            stats->rtp_min_gap_per_frame = stats->rtp_latest_gap_per_frame;
        }

        if(interval_diff > stats->frame_max_interval){
            stats->frame_max_interval = interval_diff;
        }

        if(interval_diff < stats->frame_min_interval){
            stats->frame_min_interval = interval_diff;
        }

        if(stats->rtp_packets_count_per_frame > MAX_RTP_PACKETS){
            stats->packet_distribution_overflow++;
            printf("\033[1;31mpackets per frame statistic overflow: %u > %u\033[0m\n", stats->rtp_packets_count_per_frame, MAX_RTP_PACKETS);
        }else{
            stats->packet_distribution[stats->rtp_packets_count_per_frame]++;
        }
        stats->rtp_packets_previous_count   = stats->rtp_packets_count_per_frame;

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
    uint32_t frame_total = stats->frame_data_ontime_count+stats->frame_data_delay_count;
    printf("\n");
    printf("-- Summary ---------------\n");
    print_packets_delivery_histogram(stats);
    printf("--------------------------\n");
    print_packets_gap_histogram(stats);
    printf("--------------------------\n");
    printf("Max frame interval time: %e us -  %.02e Hz\n", stats->frame_max_interval, 1000000/stats->frame_max_interval);
    printf("Min frame interval time: %e us -  %.02e Hz\n", stats->frame_min_interval, 1000000/stats->frame_min_interval);
    printf("    Frame interval time: %e us -  %.02e Hz\n", stats->frame_estimate_interval, 1000000/stats->frame_estimate_interval);
    printf("  Frame presume delayed: %u\n", stats->frame_data_delay_count);
    printf("    Frame on-time count: %u\n", stats->frame_data_ontime_count);
    if (0 != frame_total) {
        printf("          Frame on-time: %u%%\n", 100*stats->frame_data_ontime_count/(stats->frame_data_ontime_count+stats->frame_data_delay_count));
    }
    printf("--------------------------\n");
    printf("    Total max RTP packets: %u\n", stats->rtp_max_packets_per_frame);
    printf("    Total min RTP packets: %u\n", stats->rtp_min_packets_per_frame);
    printf("  Total valid RTP packets: %u\n", stats->valid_count);
    printf("Total invalid RTP packets: %u\n", stats->invalid_count);
    printf("    Max RTP packet length: %u\n", stats->max_recv_len);
    printf("    Min RTP packet length: %u\n", stats->min_recv_len);
    printf("  Max RTP packet overflow: %u\n", stats->packet_distribution_overflow);
    printf("     RTP packet threshold: %u\n", stats->rtp_packets_safe_threshold);
    printf("      RTP packet max peak: %u\n", stats->rtp_packets_max_peak_bucket);
    printf("--------------------------\n");
    printf(" RTP send buffer max size: %d bytes\n", stats->send_buffer_max);
    printf("   RTP send time max diff: %ld us\n", stats->send_time_max);
    printf("   RTP recv time max diff: %ld us\n", stats->recv_time_max);

    printf("        RTP sync max diff: %ld us\n", stats->rtp_sync_time_max);
    printf("      RTP pack a max diff: %ld us\n", stats->rtp_pack_a_time_max);
    printf("      RTP pack b max diff: %ld us\n", stats->rtp_pack_b_time_max);
    printf("      RTP pack c max diff: %ld us\n", stats->rtp_pack_c_time_max);
  
    printf("--------------------------\n");
    printf("    Total IMU RTP packets: %u\n", stats->rtp_imu_count);
    printf("     Max IMU packet count: %u\n", stats->rtp_max_imu_per_frame);
    printf("     Min IMU packet count: %u\n", stats->rtp_min_imu_per_frame);

    printf("Total IMU+IMG RTP packets: %u\n", stats->rtp_imu_plus_img_count);
    printf(" Total IMU in RTP dropped: %u\n", stats->rtp_imu_in_img_droped);
    printf("Total IMU in RTP transfer: %u\n", stats->rtp_imu_in_img_transfered);
    printf(" Max IMU+IMG packet count: %u\n", stats->rtp_max_imu_plus_img_per_frame);
    printf(" Min IMU+IMG packet count: %u\n", stats->rtp_min_imu_plus_img_per_frame);

    printf("Total IMU+INV RTP packets: %u\n", stats->rtp_imu_invalid_img_count);
    printf(" Max IMU+INV packet count: %u\n", stats->rtp_max_imu_invalid_img_per_frame);
    printf(" Min IMU+INV packet count: %u\n", stats->rtp_min_imu_invalid_img_per_frame);
    printf("--------------------------\n");
    print_rtp_packet_histogram(stats);
    printf("--------------------------\n");
    print_fps_histogram(stats);
    printf("--------------------------\n");
}

bool is_first_packet_of_frame(uint16_t current_seq, bool marker_bit, bool* packet_lost) {
    // Static variables to store the previous sequence number and marker bit
    static uint16_t previous_seq = 0;
    static bool previous_marker_bit = false;
    static bool first_packet_checked = false;

    // Check if this is the first packet of a new frame
    bool is_first_packet = false;
    *packet_lost = false;

    // Detect packet loss (if previous sequence number + 1 != current sequence number)
    if (first_packet_checked) {
        // Check for packet loss, considering wrap-around
        if ((current_seq != (previous_seq + 1)) && !(previous_seq == 65535 && current_seq == 0)) {
            *packet_lost = true;
            //printf("\033[31mPacket loss detected! Expected sequence number: %d, but got: %d\033[0m\n", (previous_seq + 1) % 65536, current_seq);
        }
    }

    // If the current packet has marker bit set to 0 and previous packet had marker bit set to 1,
    // it's likely the first packet of a new frame
    if (previous_marker_bit == true && marker_bit == false) {
        is_first_packet = true;
    }

    // Update the static variables for the next call
    previous_seq = current_seq;
    previous_marker_bit = marker_bit;
    first_packet_checked = true;  // Mark that the first packet has been checked

    return is_first_packet;
}
