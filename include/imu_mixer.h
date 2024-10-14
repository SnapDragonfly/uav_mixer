#ifndef IMU_MIXER_H
#define IMU_MIXER_H

#define MAGIC_IMU_FRAME_NUM 0xFFFF
#define MAGIC_CLI_FRAME_NUM 0xFFFE

typedef union {
    struct {
        uint32_t img_sec;          // Image seconds
        uint32_t img_nsec;         // Image nanoseconds
        uint32_t img_timestamp;    // Image timestamp
    };
    
    struct {
        uint32_t base_sec;         // Base seconds
        uint32_t base_nsec;        // Base nanoseconds
        uint32_t base_timestamp;    // Base timestamp
    };
} mix_timestamp_t;

typedef struct {
    mix_timestamp_t timestamp;   // Common timestamp fields

    /* number of IMU data */
    uint16_t imu_num;            // Number of IMU data
    uint16_t reserved;           // Reserved for future use
} mix_head_t;

typedef struct {
    uint32_t imu_sec;          // Timestamp seconds
    uint32_t imu_nsec;         // Timestamp nanoseconds
    float xacc;                // Linear acceleration X
    float yacc;                // Linear acceleration Y
    float zacc;                // Linear acceleration Z
    float xgyro;               // Angular velocity X
    float ygyro;               // Angular velocity Y
    float zgyro;               // Angular velocity Z
    float q_w;                 // Quaternion W
    float q_x;                 // Quaternion X
    float q_y;                 // Quaternion Y
    float q_z;                 // Quaternion Z
} imu_data_t;

#endif