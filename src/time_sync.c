#include <stdio.h>
#include <stdlib.h>
#include <time.h>
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
        sys->sync_times[i].tv_nsec = 0;
        sys->sync_counts[i] = 0;
    }
    sys->sync_ok = false;
}

// Function to increase sync clock to reduce err of SyncSystem
void inc_sync_clock(sync_time_t *sys) {
    sys->clock_hz += RTP_CLOCK_INC_UNIT_HZ;
    //printf("+\n");
}

// Function to decrease sync clock to reduce err of SyncSystem
void dec_sync_clock(sync_time_t *sys) {
    sys->clock_hz -= RTP_CLOCK_DEC_UNIT_HZ;
    //printf("-\n");
}

bool get_sync_status(sync_time_t *sys) {
    return sys->sync_ok;
}

bool set_sync_status(sync_time_t *sys, bool status) {
    sys->sync_ok = status;
    return sys->sync_ok;
}

// Function to synchronize system time with the given count
void synchronize_time(sync_time_t *sys, double count) {
    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time); // Get the current system time

    // Store the system time and count in the circular buffer
    sys->sync_times[sys->sync_index] = current_time;
    sys->sync_counts[sys->sync_index] = count;
    
    sys->sync_index = (sys->sync_index + 1) % MAX_TIME_SYNC_SAMPLES;
    if (sys->sample_count < MAX_TIME_SYNC_SAMPLES) {
        sys->sample_count++;
    }
}

struct timespec* estimate_time(sync_time_t *sys, struct timespec* estimated_time, double count) {

    estimated_time->tv_sec  = 0;
    estimated_time->tv_nsec = 0;

    if (sys->sample_count == 0) {
        return estimated_time; // No sync data available
    }

    // Use the most recent sync data for estimation
    int recent_index = (sys->sync_index - 1 + MAX_TIME_SYNC_SAMPLES) % MAX_TIME_SYNC_SAMPLES;
    struct timespec sync_time = sys->sync_times[recent_index];
    double sync_count = sys->sync_counts[recent_index];

    // Calculate time difference between the given count and the synced count
    double count_diff = count - sync_count;

    // Debug output
    //printf("count: %.2f, sync_count: %.2f, count_diff: %.2f\n", count, sync_count, count_diff);
    
    // Convert count difference to time difference (in microseconds)
    double estimated_time_diff = (count_diff / sys->clock_hz) * 1000000.0; // Convert to µs

    // Debug output
    //printf("Sync time - sec: %ld, nsec: %ld\n", sync_time.tv_sec, sync_time.tv_nsec);
    //printf("estimated_time_diff: %.2f µs\n", estimated_time_diff);

    // Calculate the estimated system time
    *estimated_time = sync_time;
    //estimated_time->tv_nsec += estimated_time_diff * 1000.0; // Convert µs to ns
    long long tmp_ns = estimated_time->tv_nsec + estimated_time_diff * 1000.0;

    // Handle overflow in nanoseconds field
    if (tmp_ns >= 1000000000L) {
        estimated_time->tv_sec += tmp_ns / 1000000000L;
        estimated_time->tv_nsec = tmp_ns % 1000000000L;
    }
    //while (estimated_time->tv_nsec >= 1000000000L) {
    //    estimated_time->tv_sec += estimated_time->tv_nsec / 1000000000L;
    //    estimated_time->tv_nsec = estimated_time->tv_nsec % 1000000000L;
    //}

    // Debug output
    //printf("Estimated time - sec: %ld, nsec: %ld\n", estimated_time->tv_sec, estimated_time->tv_nsec);

    // Return the estimated system time in microseconds since epoch
    return estimated_time;
}

// Function to calculate the count based on the current system time
uint32_t calculate_timestamp(sync_time_t *sys) {
    if (sys->sample_count == 0) {
        return 0; // No sync data available, return 0 for unsigned int
    }

    // Use the most recent sync data for the reverse calculation
    int recent_index = (sys->sync_index - 1 + MAX_TIME_SYNC_SAMPLES) % MAX_TIME_SYNC_SAMPLES;
    struct timespec sync_time = sys->sync_times[recent_index];
    double sync_count = sys->sync_counts[recent_index];

    // Get the current system time
    struct timespec current_time;
    clock_gettime(CLOCK_REALTIME, &current_time);

    // Calculate time difference (in microseconds) between the current time and sync_time
    double time_diff = (current_time.tv_sec - sync_time.tv_sec) * 1000000.0 + 
                       (current_time.tv_nsec - sync_time.tv_nsec) / 1000.0;

    // Convert time difference back to count difference using the clock frequency (convert from µs to clock cycles)
    double count_diff = (time_diff / 1000000.0) * sys->clock_hz;

    // Debug output
    //printf("Base time - sec: %ld, nsec: %ld\n", sync_time.tv_sec, sync_time.tv_nsec);
    //printf("Curr time - sec: %ld, nsec: %ld\n", current_time.tv_sec, current_time.tv_nsec);
    //printf("Calculated count diff: %f time diff: %f stamp: %f\n", count_diff, time_diff, count_diff + time_diff);

    // Calculate the estimated count and cast to uint32_t
    return (uint32_t)(sync_count + count_diff);
}

// Function to get the current system time in microseconds as a double
struct timespec get_system_time_us() {
    struct timespec current_time;

    // Get the current time
    clock_gettime(CLOCK_REALTIME, &current_time);

    // Convert time to microseconds and return as double
    return current_time;
}

bool is_before(struct timespec* estimated_time, struct timespec* current_time) {
    // Compare seconds first
    if (estimated_time->tv_sec < current_time->tv_sec) {
        return true;
    } else if (estimated_time->tv_sec > current_time->tv_sec) {
        return false;
    }

    // If seconds are equal, compare nanoseconds
    if (estimated_time->tv_nsec < current_time->tv_nsec) {
        return true;
    } else {
        return false;
    }
}

struct timespec* time_minus_us(struct timespec *time, uint32_t us) {
    long interval_nanos = us * 1000;
    time->tv_nsec -= interval_nanos;

    if (time->tv_nsec < 0) {
        time->tv_nsec += 1000000000;
        time->tv_sec -= 1;
    }

    if (time->tv_sec < 0) {
        time->tv_sec = 0;
        time->tv_nsec = 0;
    }

    return time;
}
