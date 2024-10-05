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

#define FORWARD_BUF_LEN          4094               //uav_mixer settings


// RTP FPS default configuration
#define RTP_FPS_RATE             30                 //Hz

#define MAX_RTP_PACKETS          200                //uav_mixer settings
#define MAX_FRAME_PER_SECOND     120                //uav_mixer settings
#define SCALE_FACTOR             50                 //uav_mixer settings

// RTP Time Sync default configuration
#define RTP_TIME_CLOCK_HZ        90000              //uav_mixer settings
#define RTP_FRAME_SYNC_NUM       200                //uav_mixer settings
#define RTP_FRAME_SYNC_THRESHOLD 10                 //uav_mixer settings
#define RTP_FPS_UPDATE_RATE      (RTP_FPS_RATE*20)  //uav_mixer settings

#define MAX_TIME_SYNC_SAMPLES    10                 //uav_mixer settings

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

#endif /* UAV_MIXER_CONFIG_H */