#ifndef __SEEKER_TYPE_HPP__
#define __SEEKER_TYPE_HPP__

#include <stdint.h>
#include <vector>
#include <atomic>
#include <string>
#include <functional>
#include <thread>
#include <mutex>
#include <iostream>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <fstream>
#include <vector>
#include <map>

// namespace SEEKER {

#define MAKE_UNIQUE_ID(major, sub, a, b) ((major<<24) | (sub<<16) | (a<<8) | (b))

typedef enum _dev_type {
    SEEKER_DEV1_NONE             = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x00),
    SEEKER_DEV1_STEREO_SHORT     = MAKE_UNIQUE_ID('C', 0x02, 'S', 0x02),
    SEEKER_DEV1_STEREO_LONG      = MAKE_UNIQUE_ID('C', 0x02, 'L', 0x03),
    SEEKER_DEV1_STEREO_DUAL      = MAKE_UNIQUE_ID('C', 0x04, 'D', 0x05),
    SEEKER_DEV1_OMNI_DEPTH       = MAKE_UNIQUE_ID('C', 0x08, 'O', 0x06),
} dev_type_t;

typedef enum _dev_state {
    STATE_RESET,  // 复位状态
    STATE_NORMAL, // 正常状态。可以抓图，深度图等
    STATE_CONFIG, // 配置状态，
    STATE_UPDATE, // 升级状态，不能抓图
} dev_state_t;

// 设备能力
typedef enum {
    DEV_CAPABILITY_RGB            = 0x00000001,
    DEV_CAPABILITY_DEPTH          = 0x00000002,
    DEV_CAPABILITY_SENSOR         = 0x00000004,
    DEV_CAPABILITY_IR             = 0x00000008,
    DEV_CAPABILITY_TOF3D          = 0x00000010,
    DEV_CAPABILITY_SPECKLE        = 0x00000020,
    DEV_CAPABILITY_WRITE_FLASH    = 0x00000040,
    DEV_CAPABILITY_IR_FLASH       = 0x00000080,
} dev_capability_t;

// 传感器能力
typedef enum {
    SENSOR_CAPABILITY_ACC            = 0x00000001,
    SENSOR_CAPABILITY_GYRO           = 0x00000002,
    SENSOR_CAPABILITY_MAG            = 0x00000004,
    SENSOR_CAPABILITY_PRESSURE       = 0x00000008,
    SENSOR_CAPABILITY_GPS            = 0x00000040,
    SENSOR_CAPABILITY_ACC2           = 0x00000010,
    SENSOR_CAPABILITY_GYRO2          = 0x00000020,
    SENSOR_CAPABILITY_TOF_POINT      = 0x00000080,
    SENSOR_CAPABILITY_TOF_3D         = 0x00000100,
    SENSOR_CAPABILITY_OPTICAL_FLOW   = 0x00000200,
} sensor_capability_t;

// rgb depth tof 
typedef struct _dev_info {
    /** 摄像头 **/
    uint32_t rgb_resolution_width; // 分辨率
    uint32_t rgb_resolution_height; // 分辨率
    uint8_t rgb_camera_number; // 图像拆分数量

    /** 深度图 **/
    uint32_t depth_resolution_width; // 分辨率
    uint32_t depth_resolution_height; // 分辨率
    uint8_t depth_camera_number; // 图像拆分数量

    /** 传感器 **/
    uint64_t sensor_capability; // 传感器类型

    /** 保留 **/
} __attribute__((packed)) dev_info_t;

typedef enum {
    CAMERA_MODEL_NONE = 0,
    CAMERA_MODEL_DISTORTEDPINHOLE = 0,  // pinhole-radtan
    CAMERA_MODEL_EQUIDISTANTPINHOLE,    // pinhole-equidistant
    CAMERA_MODEL_FOVPINHOLE,            // pinhole-fov
    CAMERA_MODEL_OMNI,                  // omni-none
    CAMERA_MODEL_DISTORTEDOMNI,         // omni-radtan
    CAMERA_MODEL_EXTENDEDUNIFIED,       // eucm-none
    CAMERA_MODEL_DOUBLESPHERE,          // ds-none
    CAMERA_MODEL_OTHER,
} camera_models_t;

const std::pair<std::string, std::string> cameraModels_to_string[] = {
    {"pinhole", "radtan"},
    {"pinhole", "equidistant"},
    {"pinhole", "fov"},
    {"omni",    "none"},
    {"omni",    "radtan"},
    {"eucm",    "none"},
    {"ds",      "none"},
};

// 发送和接收的包
// 对外和对内两套，兼容，但是内部多几个
typedef enum _dev_event_type {
    //      | input/output/dual | custom/mavlink | version now 0x01 | msg id |
    //                                       I/O/D  C/M , version
    EVENT_TYPE_ANY           = MAKE_UNIQUE_ID(0x00,0x00,0x01, 0x00),
    EVENT_TYPE_POSE_STAMPED  = MAKE_UNIQUE_ID('I', 'M', 0x01, 0x01), // 位置点（输入）
    EVENT_TYPE_ODOMETRY      = MAKE_UNIQUE_ID('I', 'M', 0x01, 0x02), // 里程计（输入）
    EVENT_TYPE_SENSOR_IMU    = MAKE_UNIQUE_ID('I', 'M', 0x01, 0x03), // 传感器（输入）
    EVENT_TYPE_SENSOR_CUSTOM = MAKE_UNIQUE_ID('I', 'C', 0x01, 0x04), // 传感器（输入）
    EVENT_TYPE_UVC_INFO      = MAKE_UNIQUE_ID('I', 'C', 0x01, 0x10), // 摄像头时间戳（输入）
    EVENT_TYPE_FEATURE       = MAKE_UNIQUE_ID('I', 'C', 0x01, 0x05), // 传感器（输入）
    EVENT_TYPE_TIME_SYNC     = MAKE_UNIQUE_ID('D', 'C', 0x01, 0x06), // 时钟同步（输入/输出）
    EVENT_TYPE_CLI_CAM       = MAKE_UNIQUE_ID('D', 'C', 0x01, 0x07), // 摄像头标定参数（输入/输出）
    EVENT_TYPE_CLI_DEPTH     = MAKE_UNIQUE_ID('D', 'C', 0x01, 0x08), // 深度图标定参数（输入/输出）
    // EVENT_TYPE_OTADATA       = MAKE_UNIQUE_ID('O', 'C', 0x01, 0x09), // OTA升级数据（输出）
    EVENT_TYPE_REBOOT        = MAKE_UNIQUE_ID('O', 'C', 0x01, 0x0a), // 摄像头复位（输出）
    // EVENT_TYPE_CLI_TOF; // TOF传感器标定参数
} dev_event_type_t;

typedef struct {
    uint64_t reserve;
    uint64_t sec;
    uint64_t nsec;
    uint32_t seq;
    uint32_t reserve2;
} __attribute__((packed)) event_header_t;

typedef struct {
    event_header_t header;
    int ack;
} __attribute__((packed)) event_ack_t;

// /mavros/vision_pose/pose_cov
// geometry_msgs/PoseStamped
typedef struct {
    event_header_t header;
    dev_type_t  dev_type; // 设备型号
    dev_state_t dev_state; // 设备状态

    dev_capability_t dev_capability;
    sensor_capability_t sensor_capability;

    char dev_firmware_version[32]; // 固件版本信息

    uint16_t rgb_resolution_width;
    uint16_t rgb_resolution_height;
    uint16_t rgb_split_number;

    uint16_t depth_resolution_width;
    uint16_t depth_resolution_height;
    uint16_t depth_split_number;
} __attribute__((packed)) event_capability_t;

typedef enum {
    VALID_FLAG_BIT_POSITION_XY              = (1<<0),
    VALID_FLAG_BIT_POSITION_Z               = (1<<1),
    VALID_FLAG_BIT_ORIENTATION              = (1<<2),
    VALID_FLAG_BIT_ORIENTATION_CONV         = (1<<3),
    VALID_FLAG_BIT_ANGULAR_VELOCITY         = (1<<4),
    VALID_FLAG_BIT_ANGULAR_VELOCITY_CONV    = (1<<5),
    VALID_FLAG_BIT_LINEAR_ACCELERATION      = (1<<6),
    VALID_FLAG_BIT_LINEAR_ACCELERATION_CONV = (1<<7),
    VALID_FLAG_BIT_MAGNETIC_FIELD           = (1<<8),
    VALID_FLAG_BIT_MAGNETIC_FIELD_CONV      = (1<<9),
    VALID_FLAG_BIT_GPS_XY                   = (1<<10),
    VALID_FLAG_BIT_GPS_Z                    = (1<<11),
} sensor_valid_flags_t;

// /mavros/vision_pose/pose_cov
// geometry_msgs/PoseStamped
typedef struct {
    event_header_t header;
    uint64_t valid_flag;
    double position_x;
    double position_y;
    double position_z;
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double covariance[36];
} __attribute__((packed)) event_pose_stamped_t;

// /mavros/odometry/out
// nav_msgs/Odometry
typedef struct {
    event_header_t header;
    uint64_t valid_flag;
    double position_x;
    double position_y;
    double position_z;
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double pose_covariance[36];
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double twist_covariance[36];
} __attribute__((packed)) event_odometry_t;

// /mavros/imu/data
// /mavros/imu/data_raw
// sensor_msgs/Imu
typedef struct {
    event_header_t header;
    uint64_t valid_flag;
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_covariance[9];
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double angular_velocity_covariance[9];
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
    double linear_acceleration_covariance[9];
} __attribute__((packed)) event_sensor_imu_t;

typedef struct {
    event_header_t header;
    uint64_t valid_flag;
    double orientation_w;
    double orientation_x;
    double orientation_y;
    double orientation_z;
    double orientation_covariance[9];
    double angular_velocity_x;
    double angular_velocity_y;
    double angular_velocity_z;
    double angular_velocity_covariance[9];
    double linear_acceleration_x;
    double linear_acceleration_y;
    double linear_acceleration_z;
    double linear_acceleration_covariance[9];
    double magnetic_field_x;
    double magnetic_field_y;
    double magnetic_field_z;
    double magnetic_field_covariance[9];
    double pressure;
    double temperature;
} __attribute__((packed)) event_sensor_custom_t;

typedef struct _event_uvc_info {
    event_header_t header;
    // todo
    // double exp;
    // int64_t ISO;
    // double awb;
} __attribute__((packed)) event_uvc_info_t;

typedef struct _event_point {
    int16_t feature_id;
    uint16_t u;
    uint16_t v;
} __attribute__((packed)) event_point_t;

typedef struct _event_feature {
    event_header_t header;
    int cam_id;
    event_point_t point[250];
} __attribute__((packed)) event_feature_t;

// 主从机间同步时间
typedef struct _event_time_sync {
    event_header_t header;
    double host_send_timestamp; // 主机发送命令的时间（主机的本地时间）
    double slave_recv_timestamp; // 从机接收到命令的时间（从机的本地的时间）
    double slave_send_timestamp; // 从机发送命令的时间（从机的本地时间）
    double host_recv_timestamp; // 主机接收到命令的时间（主机的本地的时间）
} __attribute__((packed)) event_time_sync_t;

typedef struct _cam_cali {
    uint8_t cam_id;
    camera_models_t camera_model;
    double T_cam_imu_se3_qw;
    double T_cam_imu_se3_qx;
    double T_cam_imu_se3_qy;
    double T_cam_imu_se3_qz;
    double T_cam_imu_se3_x;
    double T_cam_imu_se3_y;
    double T_cam_imu_se3_z;
    double T_cn_cnm1_se3_qw;
    double T_cn_cnm1_se3_qx;
    double T_cn_cnm1_se3_qy;
    double T_cn_cnm1_se3_qz;
    double T_cn_cnm1_se3_x;
    double T_cn_cnm1_se3_y;
    double T_cn_cnm1_se3_z;
    double distortion_coeffs_k1;
    double distortion_coeffs_k2;
    double distortion_coeffs_p1;
    double distortion_coeffs_p2;
    double distortion_coeffs_k3;
    double intrinsics_xi;
    double intrinsics_fx;
    double intrinsics_fy;
    double intrinsics_cx;
    double intrinsics_cy;
    uint32_t resolution_width;
    uint32_t resolution_height;
} __attribute__((packed)) event_cam_cali_t;

typedef struct _dev_cali {
    event_header_t header;
    uint64_t cali_flag; // reserve
    uint32_t fx;
    uint32_t fy;
    event_cam_cali_t cam[8];
} __attribute__((packed)) event_dev_cali_t;

// 固件升级的实时状态
typedef enum _update_status {
    OTA_STATUS_FINISHED   = 1, // 升级完成
    OTA_STATUS_RUNNING    = 2, // 正在升级
    OTA_STATUS_FAILED     = 3, // 升级失败
    OTA_STATUS_UNKNOWN    = 4, // 升级失败(未知错误)
    OTA_STATUS_ERROR_DATA = 5, // 升级失败(固件包错误)
    OTA_STATUS_IO         = 6, // 升级失败(IO读写失败)
} update_status_t;

typedef struct _event_ota {
    update_status_t status;   // 升级的状态
    uint8_t progress;         // 升级进度 0到100
} __attribute__((packed)) event_ota_t;

typedef struct _event_reboot{
    char reboot[20]; // update reboot
} __attribute__((packed)) event_reboot_t;

typedef union {
    event_capability_t    capability;
    event_pose_stamped_t  pose;
    event_odometry_t      odom;
    event_sensor_imu_t    sensor;
    event_sensor_custom_t sensor_custom;
    event_uvc_info_t      uvc_info;
    event_feature_t       feature;
    event_time_sync_t     sync;
    event_dev_cali_t      cali;
    event_ota_t           ota;
    event_reboot_t        reboot;
    // char  pad[2040];
} __attribute__((packed)) dev_event_data_t;

enum {
    SEEKER_PROTOCOL_NONE = 0,
    SEEKER_PROTOCOL_CMD_SET,
    SEEKER_PROTOCOL_CMD_GET,
    SEEKER_PROTOCOL_EVT,
};

typedef struct _deviceevent {
    uint8_t protocol_version;
    uint8_t protocol_type;
    uint8_t reverse1;
    uint8_t reverse2;
    dev_event_type_t type;
    dev_event_data_t event;

    friend std::ostream& operator<<(std::ostream& os, const _deviceevent& data) {
        return os.write(reinterpret_cast<const char*>(&data), sizeof(_deviceevent));
    }
    friend std::istream& operator>>(std::istream& is, _deviceevent& data) {
        return is.read(reinterpret_cast<char*>(&data), sizeof(_deviceevent));
    }
} __attribute__((packed)) device_event_t;


typedef struct seeker_device_{
    uint16_t video_index;
    uint16_t devnum;
    uint16_t busnum;
    uint16_t idVendor;
    uint16_t idProduct;
    std::string serial;
    std::string product;
    dev_type_t dev_type;
    dev_info_t dev_info;

    friend std::ostream& operator<<(std::ostream &os,const seeker_device_ &y){
        os  << "[video" << std::dec << y.video_index << "]:"
            << "devnum:" << y.devnum
            << " busnum:" << y.busnum 
            << " idVendor:" << std::hex << y.idVendor
            << " idProduct:"<< y.idProduct
            << " serial:" << y.serial 
            << " product:" << y.product << std::dec
            << std::endl
            << " rgb:num:" << (int)y.dev_info.rgb_camera_number
            << " rgb:width:" << y.dev_info.rgb_resolution_width
            << " rgb:height:" << y.dev_info.rgb_resolution_height
            << " depth:num:" << (int)y.dev_info.depth_camera_number
            << " depth:width:" << y.dev_info.depth_resolution_width
            << " depth:height:" << y.dev_info.depth_resolution_height
            << " sensors:" << std::hex << y.dev_info.sensor_capability << std::dec;
        return os;
    }
} seeker_device_t;

#endif // __SEEKER_TYPE_HPP__
