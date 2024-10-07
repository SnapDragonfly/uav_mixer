#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>
#include <stdbool.h>

#include "config.h"

// Structure to hold synchronization data
typedef struct {
    double count;                            // Count value during synchronization
    double clock_hz;                         // Clock frequency in Hz
    struct timespec sync_times[MAX_TIME_SYNC_SAMPLES];  // System times for sync points
    double sync_counts[MAX_TIME_SYNC_SAMPLES];         // Counts for sync points
    int sync_index;                          // Index for the circular buffer
    int sample_count;                        // Count of samples added
    bool sync_ok;
} sync_time_t;

// Function to initialize the SyncSystem
void init_sync_system(sync_time_t *sys, double clock_hz);

void inc_sync_clock(sync_time_t *sys);
void dec_sync_clock(sync_time_t *sys);

bool set_sync_status(sync_time_t *sys, bool status);
bool get_sync_status(sync_time_t *sys);

// Function to synchronize time with the given count
void synchronize_time(sync_time_t *sys, double count);

// Function to estimate system time based on count and clock frequency
struct timespec* estimate_time(sync_time_t *sys, struct timespec* estimated_time, double count);

// Function to calculate the count based on the current system time
uint32_t calculate_timestamp(sync_time_t *sys);

// Function to get the current system time in microseconds as a double
struct timespec get_system_time_us();

struct timespec* time_minus_us(struct timespec *time, uint32_t us);

#endif /* TIME_SYNC_H */