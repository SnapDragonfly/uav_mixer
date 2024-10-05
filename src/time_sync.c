#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "time_sync.h"

// Function to initialize the SyncSystem
void init_sync_system(sync_time_t *sys, double clock_hz) {
    sys->count = 0;
    sys->clock_hz = clock_hz;
    sys->sync_index = 0;
    sys->sample_count = 0;
    for (int i = 0; i < MAX_TIME_SYNC_SAMPLES; i++) {
        sys->sync_times[i].tv_sec = 0;
        sys->sync_times[i].tv_usec = 0;
        sys->sync_counts[i] = 0;
    }
}

// Function to synchronize system time with the given count
void synchronize_time(sync_time_t *sys, double count) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL); // Get the current system time

    // Store the system time and count in the circular buffer
    sys->sync_times[sys->sync_index] = current_time;
    sys->sync_counts[sys->sync_index] = count;
    
    sys->sync_index = (sys->sync_index + 1) % MAX_TIME_SYNC_SAMPLES;
    if (sys->sample_count < MAX_TIME_SYNC_SAMPLES) {
        sys->sample_count++;
    }
}


// Function to estimate system time based on a new count
double estimate_time(sync_time_t *sys, double count) {
    if (sys->sample_count == 0) {
        return -1; // No sync data available
    }

    // Use the most recent sync data for estimation
    int recent_index = (sys->sync_index - 1 + MAX_TIME_SYNC_SAMPLES) % MAX_TIME_SYNC_SAMPLES;
    struct timeval sync_time = sys->sync_times[recent_index];
    double sync_count = sys->sync_counts[recent_index];

    // Calculate time difference between the given count and the synced count
    double count_diff = count - sync_count;
    
    // Convert count difference to time difference (in microseconds)
    double estimated_time_diff = (count_diff / sys->clock_hz) * 1000000.0; // Convert to µs

    // Calculate the estimated system time
    struct timeval estimated_time = sync_time;
    estimated_time.tv_usec += estimated_time_diff;

    // Handle overflow in microseconds field
    while (estimated_time.tv_usec >= 1000000) {
        estimated_time.tv_sec += 1;
        estimated_time.tv_usec -= 1000000;
    }

    // Return the estimated system time in microseconds since epoch
    return estimated_time.tv_sec * 1000000.0 + estimated_time.tv_usec;
}

// Function to calculate the count based on the current system time
uint32_t calculate_timestamp(sync_time_t *sys) {
    if (sys->sample_count == 0) {
        return 0; // No sync data available, return 0 for unsigned int
    }

    // Use the most recent sync data for the reverse calculation
    int recent_index = (sys->sync_index - 1 + MAX_TIME_SYNC_SAMPLES) % MAX_TIME_SYNC_SAMPLES;
    struct timeval sync_time = sys->sync_times[recent_index];
    double sync_count = sys->sync_counts[recent_index];

    // Get the current system time
    struct timeval current_time;
    gettimeofday(&current_time, NULL);

    // Calculate time difference (in microseconds) between the current time and sync_time
    double time_diff = (current_time.tv_sec - sync_time.tv_sec) * 1000000.0 + (current_time.tv_usec - sync_time.tv_usec);

    // Convert time difference back to count difference using the clock frequency (convert from µs to clock cycles)
    double count_diff = (time_diff / 1000000.0) * sys->clock_hz;

    // Calculate the estimated count and cast to uint32_t
    return (uint32_t)(sync_count + count_diff);
}

// Function to calculate error between the estimated time and actual system time
double calculate_error(sync_time_t *sys, double count) {
    struct timeval current_time;
    gettimeofday(&current_time, NULL); // Get the current system time

    // Estimate the system time for the given count
    double estimated_time = estimate_time(sys, count);

    // If estimation failed
    if (estimated_time < 0) {
        return -1;
    }

    // Get the actual current/last sync time in microseconds since epoch
    double actual_time = current_time.tv_sec * 1000000.0 + current_time.tv_usec;
    int recent_index = (sys->sync_index - 1 + MAX_TIME_SYNC_SAMPLES) % MAX_TIME_SYNC_SAMPLES;
    struct timeval sync_time = sys->sync_times[recent_index];
    double elapsed_time = actual_time - sync_time.tv_sec * 1000000.0 - sync_time.tv_usec;

    // Calculate the error (difference between actual and estimated time)
    double error = actual_time - estimated_time; 
    if (elapsed_time == error){ //just sync before error calculate_error
        error = 0;
    }
    //printf("actual %.2f estimate %.2f elapse %.2f err %.2f\n", actual_time, estimated_time, elapsed_time, error);
    
    // Calculate the change needed in parts per million (ppm) relative to clock frequency
    double percentage = error / elapsed_time * 100.0; // ppm relative to clock frequency

#if UAV_MIXER_DEBUG
    // Print whether to increase or decrease the clock frequency
    if (error > 0) {
        printf("Decrease clock frequency by %.2f %% \n", percentage);
    } else if (error < 0) {
        printf("Increase clock frequency by %.2f %% \n", percentage);
    } else {
        printf("Clock frequency is accurate.\n");
    }
#endif
    return percentage;
}

// Function to get the current system time in microseconds as a double
double get_system_time_us() {
    struct timeval current_time;

    // Get the current time
    gettimeofday(&current_time, NULL);

    // Convert time to microseconds and return as double
    double time_in_us = (double)(current_time.tv_sec) * 1e6 + (double)(current_time.tv_usec);
    return time_in_us;
}
