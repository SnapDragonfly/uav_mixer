#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#include "imu_mixer.h"
#include "config.h"

typedef struct {
    imu_data_t buffer[MAX_RING_BUFFER_SIZE];
    uint32_t head;
    uint32_t tail;
    uint32_t count;
    uint32_t max_usage; // To track the maximum usage percentage
} ring_buffer_t;

// Function prototypes
void init_rb(ring_buffer_t *rb);
bool push_rb(ring_buffer_t *rb, const imu_data_t *data);
bool pop_rb(ring_buffer_t *rb, imu_data_t *data);
void print_rb_stats(ring_buffer_t *rb);
void reset_rb_max_usage(ring_buffer_t *rb);
uint32_t get_rb_count(ring_buffer_t *rb);

#endif // RING_BUFFER_H
