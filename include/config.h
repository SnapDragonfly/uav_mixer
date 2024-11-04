#ifndef UAV_MIXER_CONFIG_H
#define UAV_MIXER_CONFIG_H

// Global configuration
#define UAV_MIXER_DEBUG          0                                  //uav_mixer settings, used by developer

// UDP-control default configuration
#define CTL_LOCAL_PORT           5401                               //uav_mixer settings
#define CTL_BUF_LEN              1024                               //uav_mixer settings

// UDP-RTP forward default configuration
#define RTP_LOCAL_PORT           5400                               //RTP Source
#define RTP_LOCAL_TO_SEC         0                                  //uav_mixer settings
#define RTP_LOCAL_TO_USEC        2000                               //uav_mixer settings

#define FORWARD_IP               "127.0.0.1"                     //Forward Destination IP
#define FORWARD_PORT             5600                               //Forward Destination Port

/*
 * WIFI_MTU  4045  // Max injected packet size including all wfb-headers. 
 * https://github.com/svpcom/wfb-ng/issues/371
 */
#define FORWARD_MTU_LEN          4045                               //uav_mixer settings, traditional IP MTU 1500
#define FORWARD_BUF_LEN          (FORWARD_MTU_LEN - 20 - 8)         //uav_mixer settings, 1500 - 20 - 8 = 1472
#define FORWARD_RTP_BUF_LEN      1400                               //uav_mixer settings, ssc30kq(1215), Pi3B(1400)

#define FORWARD_RECV_BUF_SIZE    64                                 //uav_mixer settings, MB
#define FORWARD_SEND_BUF_SIZE    64                                 //uav_mixer settings, MB

#define FORWARD_RECV_BUF_LEN     (1024*1024*FORWARD_RECV_BUF_SIZE)  //1024x1024 - M
#define FORWARD_SEND_BUF_LEN     (1024*1024*FORWARD_SEND_BUF_SIZE)  //1024x1024 - M

#define FORWARD_RTP_IMU_NUM      3                                  //uav_mixer settings, RPi3B+ 10 for try
#define FORWARD_RTP_IMG_SIZE     16                                 //sizeof(mix_head_t)
#define FORWARD_RTP_IMU_SIZE     48                                 //sizeof(imu_data_t)
#define FORWARD_RTP_IMU_LEN      (FORWARD_RTP_IMU_SIZE * FORWARD_RTP_IMU_NUM)     // 5x48=240 bytes
#define FORWARD_RTP_MIX_LEN      (FORWARD_RTP_IMG_SIZE + FORWARD_RTP_IMU_LEN)     // 240+16=256 bytes

#if FORWARD_BUF_LEN - FORWARD_RTP_MIX_LEN < FORWARD_RTP_BUF_LEN
    #error "Need enough buffer size for RTP image buffer."
#endif

// RTP FPS default configuration
#define RTP_FPS_RATE             15                 //Hz

#define MAX_RTP_PACKETS          200                //uav_mixer settings
#define MAX_FRAME_PER_SECOND     120                //uav_mixer settings
#define SCALE_FACTOR             50                 //uav_mixer settings

// RTP Time Sync default configuration
#define RTP_CLOCK_FREQ_HZ        90000              //uav_mixer settings, 90000 ~ 90KHz
#define RTP_CLOCK_INC_UNIT_HZ    10                 //uav_mixer settings
#define RTP_CLOCK_DEC_UNIT_HZ    5                  //uav_mixer settings

#define RTP_STAMP_THRESHOLD      10                  //uav_mixer settings, 10%

#define MAX_TIME_SYNC_SAMPLES    10                 //uav_mixer settings

// Compile-time check
#define RTP_FRAME_IMU_NUM        28                 //uav_mixer settings, 28x48=1344 bytes
#define RTP_FRAME_IMU_MIN_NUM    5                  //uav_mixer settings, 5x48+16=256 bytes
#if ((FORWARD_BUF_LEN - FORWARD_RTP_IMU_NUM) / FORWARD_RTP_IMU_SIZE) <= RTP_FRAME_IMU_NUM
    #error "FORWARD_BUF_LEN / FORWARD_RTP_IMU_SIZE must be greater than RTP_FRAME_IMU_NUM"
#endif

// UART IMU default configuration
#define UART_DEVICE              "/dev/ttyS2"       //device: /dev/ttyUSB0
#define UART_BAUDRATE            921600             //921600, recommended
#define UART_BUF_LEN             1024               //uav_mixer settings

#define UART_TO_SEC              1                  //uav_mixer settings
#define UART_TO_USEC             0                  //uav_mixer settings

// UART Sync
#define MAVLINK_SYNC_DOWN_THRESHOLD   -3000         //uav_mixer settings (3000us)
#define MAVLINK_SYNC_UP_THRESHOLD      3000         //uav_mixer settings (3000us)

#define MAVLINK_DEFAULT_COMP_ID  191                //uav_mixer settings
#define MAVLINK_DEFAULT_FREQ     100                //uav_mixer settings

// Test spot Latitude/Longitude/Altitude
#define TEST_SPOT_LATITUDE       303025097          //test spot, user defined
#define TEST_SPOT_LOGITUDE       1201581931         //test spot, user defined
#define TEST_SPOT_ALTITUDE       4380               //test spot, user defined

// Define the maximum size of the ring buffer
#define MAX_RING_BUFFER_SIZE     50                 //uav_mixer settings

// application param config
typedef struct {
    int mavlink_freq;
    int rtp_fps;
    int rtp_clock_freq_hz;
    int rtp_local_port;
    int rtp_local_timeout;
    char forward_ip[16];
    int forward_port;
    char uart_device[20];
    int uart_baudrate;
} uav_config_t;

#ifdef __GNUC__
#define UNUSED(x) (void)(x)
#else
#define UNUSED(x) (void)(x)  // Add different handling for other compilers if needed
#endif

#endif /* UAV_MIXER_CONFIG_H */
