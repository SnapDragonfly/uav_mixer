#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>

#include "ring_buffer.h"

static pthread_mutex_t lock; // Mutex for thread safety

void init_rb(ring_buffer_t *rb) {
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->max_usage = 0;
    pthread_mutex_init(&lock, NULL);
}

bool push_rb(ring_buffer_t *rb, const imu_data_t *data) {
    pthread_mutex_lock(&lock);

    if (rb->count >= MAX_RING_BUFFER_SIZE) {
        printf("Warning: Unable to push to buffer, maximum capacity reached!\n");
        pthread_mutex_unlock(&lock);
        return false; // Buffer is full
    }

    rb->buffer[rb->head] = *data;
    rb->head = (rb->head + 1) % MAX_RING_BUFFER_SIZE;
    rb->count++;

    // Update maximum usage
    if (rb->count > rb->max_usage) {
        rb->max_usage = rb->count;
    }

    pthread_mutex_unlock(&lock);
    return true;
}

bool pop_rb(ring_buffer_t *rb, imu_data_t *data) {
    pthread_mutex_lock(&lock);

    if (rb->count == 0) {
        pthread_mutex_unlock(&lock);
        return false; // Buffer is empty
    }

    *data = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % MAX_RING_BUFFER_SIZE;
    rb->count--;

    pthread_mutex_unlock(&lock);
    return true;
}

void print_rb_stats(ring_buffer_t *rb) {
    pthread_mutex_lock(&lock);
    
    float usage_percentage = ((float)rb->max_usage / MAX_RING_BUFFER_SIZE) * 100;
    printf("  ring_buffer_t size: %d\n", sizeof(ring_buffer_t));
    printf("     imu_data_t size: %d\n", sizeof(imu_data_t));
    printf("     imu loop buffer: %d\n", MAX_RING_BUFFER_SIZE);
    printf("Max usage percentage: %.2f%%\n", usage_percentage);
    
    pthread_mutex_unlock(&lock);
}

void reset_rb_max_usage(ring_buffer_t *rb) {
    pthread_mutex_lock(&lock);
    rb->max_usage = 0;
    pthread_mutex_unlock(&lock);
}
