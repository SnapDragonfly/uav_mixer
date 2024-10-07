#ifndef UAV_MIXER_CONFIG_H
#define UAV_MIXER_CONFIG_H

// Global configuration
#define UAV_MIXER_DEBUG          0                  //uav_mixer settings, used by developer

// UDP-control default configuration
#define CTL_LOCAL_PORT           5401               //uav_mixer settings
#define CTL_BUF_LEN              1024               //uav_mixer settings

// UDP-RTP forward default configuration
#define RTP_LOCAL_PORT           5400               //RTP Source
#define FORWARD_IP               "192.168.1.19"     //Forward Destination IP
#define FORWARD_PORT             5400               //Forward Destination Port

#define FORWARD_BUF_LEN          4096               //uav_mixer settings

#define FORWARD_RTP_IMU_NUM      10                  //uav_mixer settings, RPi3B+ 10 for try
#define FORWARD_RTP_IMG_LEN      12                  //sizeof(mix_head_t)
#define FORWARD_RTP_IMU_LEN      48                  //sizeof(imu_data_t)
#define FORWARD_RTP_MIX_LEN      (FORWARD_RTP_IMG_LEN+ FORWARD_RTP_IMU_NUM*FORWARD_RTP_IMU_LEN)
#define FORWARD_RTP_PREFIX_LEN   (FORWARD_RTP_IMU_LEN*FORWARD_RTP_IMU_NUM)

// RTP FPS default configuration
#define RTP_FPS_RATE             30                 //Hz

#define MAX_RTP_PACKETS          200                //uav_mixer settings
#define MAX_FRAME_PER_SECOND     120                //uav_mixer settings
#define SCALE_FACTOR             50                 //uav_mixer settings

// RTP Time Sync default configuration
#define RTP_CLOCK_FREQ_HZ        90000              //uav_mixer settings, ~10us
#define RTP_FRAME_SYNC_NUM       200                //uav_mixer settings
#define RTP_FRAME_SYNC_THRESHOLD 10                 //uav_mixer settings
#define RTP_FPS_UPDATE_RATE      (RTP_FPS_RATE*20)  //uav_mixer settings
#define RTP_FRAME_ADJUST_MS      600                //uav_mixer settings, RPi3B+ OV5647

#define MAX_TIME_SYNC_SAMPLES    10                 //uav_mixer settings

// Compile-time check
#define RTP_FRAME_IMU_NUM        50                 //uav_mixer settings, 25x56=25*7*8=1400 bytes
#if (FORWARD_BUF_LEN / FORWARD_RTP_IMU_LEN) <= RTP_FRAME_IMU_NUM
    #error "FORWARD_BUF_LEN / FORWARD_RTP_IMU_LEN must be greater than RTP_FRAME_IMU_NUM"
#endif

// UART IMU default configuration
#define UART_DEVICE              "/dev/ttyUSB0"     //device
#define UART_BAUDRATE            921600             //921600, recommended
#define UART_BUF_LEN             1024               //uav_mixer settings
#define UART_BUF_SLEEP_US        5000               //uav_mixer settings

#define MAVLINK_DEFAULT_COMP_ID  191                //uav_mixer settings
#define MAVLINK_DEFAULT_FREQ     100                //uav_mixer settings

// Test spot Latitude/Longitude/Altitude
#define TEST_SPOT_LATITUDE       303025097          //test spot, user defined
#define TEST_SPOT_LOGITUDE       1201581931         //test spot, user defined
#define TEST_SPOT_ALTITUDE       4380               //test spot, user defined

// Define the maximum size of the ring buffer
#define MAX_RING_BUFFER_SIZE     100                //uav_mixer settings

#endif /* UAV_MIXER_CONFIG_H */