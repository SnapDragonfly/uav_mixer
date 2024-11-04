#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "imu_mixer.h"

// Structure to hold synchronization data
typedef struct {
    double count;                            // Count value during synchronization
    double clock_hz;                         // Clock frequency in Hz
    struct timespec sync_times[MAX_TIME_SYNC_SAMPLES];  // System times for sync points
    double sync_counts[MAX_TIME_SYNC_SAMPLES];         // Counts for sync points
    int sync_index;                          // Index for the circular buffer
    int sample_count;                        // Count of samples added
    int32_t stamp_up;
    int32_t stamp_down;
} sync_time_t;

// Function to initialize the SyncSystem
void init_sync_system(sync_time_t *sys, double clock_hz, int fps);

bool is_stamp_in_threshold(sync_time_t *sys, int64_t delta);
void inc_sync_clock(sync_time_t *sys);
void dec_sync_clock(sync_time_t *sys);

// Function to synchronize time with the given count
void synchronize_time(sync_time_t *sys, double count);
void synchronize_time_ex(sync_time_t *sys, double count, struct timespec* estimated_time);

// Function to estimate system time based on count and clock frequency
struct timespec* estimate_time(sync_time_t *sys, struct timespec* estimated_time, double count);

mix_timestamp_t* get_sync_cli(sync_time_t *sys, mix_timestamp_t *mix_cal);

bool is_time_before(struct timespec* estimated_time, struct timespec* current_time);
bool is_imu_before(imu_data_t* imu, struct timespec* current_time);

// Function to calculate the count based on the current system time
uint32_t calculate_timestamp(sync_time_t *sys, struct timespec* current_time);

void calculate_time_difference(struct timespec *start, struct timespec *end, struct timespec *diff);

struct timespec* time_minus_us(struct timespec *time, uint32_t us);

long timespec_diff_us(struct timespec *start, struct timespec *end);

#endif /* TIME_SYNC_H */