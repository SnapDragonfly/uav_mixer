#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>

#include "config.h"

// Structure to hold synchronization data
typedef struct {
    double count;                            // Count value during synchronization
    double clock_hz;                         // Clock frequency in Hz
    struct timeval sync_times[MAX_TIME_SYNC_SAMPLES];  // System times for sync points
    double sync_counts[MAX_TIME_SYNC_SAMPLES];         // Counts for sync points
    int sync_index;                          // Index for the circular buffer
    int sample_count;                        // Count of samples added
} SyncSystem;

// Function to initialize the SyncSystem
void init_sync_system(SyncSystem *sys, double clock_hz);

// Function to synchronize time with the given count
void synchronize_time(SyncSystem *sys, double count);

// Function to estimate system time based on count and clock frequency
double estimate_time(SyncSystem *sys, double count);

// Function to calculate the count based on the current system time
uint32_t calculate_timestamp(SyncSystem *sys);

// Function to calculate error statistics (mean error)
double calculate_error(SyncSystem *sys, double count);

// Function to get the current system time in microseconds as a double
double get_system_time_us();

#endif /* TIME_SYNC_H */