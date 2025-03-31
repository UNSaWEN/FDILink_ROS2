// ahrs_driver.h
#ifndef FDILINK_AHRS_DRIVER_H
#define FDILINK_AHRS_DRIVER_H

#include <inttypes.h>
#include <memory>
#include <mutex>
#include <future>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <crc_table.h>
#include <Eigen/Eigen>

#include <fdilink_data_struct.h>

using namespace Eigen;

namespace FDILink {

// 常量定义使用枚举类
enum class FrameType : uint8_t {
    IMU = 0x40,
    AHRS = 0x41,
    INSGPS = 0x42,
    GEODETIC_POS = 0x5c,
    GROUND = 0xf0,
    FRAME_HEAD = 0xfc,
    FRAME_END = 0xfd
};

// 帧长度使用constexpr
constexpr size_t IMU_FRAME_LEN = 0x38;
constexpr size_t AHRS_FRAME_LEN = 0x30;
constexpr size_t INSGPS_FRAME_LEN = 0x48;
constexpr size_t GEODETIC_POS_FRAME_LEN = 0x20;

// 数学常量使用标准库
constexpr double PI = M_PI;
constexpr double DEG_TO_RAD = 0.017453292519943295;

class AhrsDriver : public rclcpp::Node {
public:
    explicit AhrsDriver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~AhrsDriver() override;

private:
    bool if_debug_;
    int sn_lost_;
    uint8_t current_serial_num_;
    bool first_serial_num_received_;
    int device_type_;

    serial::Serial serial_port_;
    std::string serial_port_name_;
    int serial_baudrate_;
    int serial_timeout_;

    FDILink::imu_frame_read imu_frame_;
    FDILink::ahrs_frame_read ahrs_frame_;
    FDILink::insgps_frame_read insgps_frame_;
    FDILink::Geodetic_Position_frame_read geodetic_pos_frame_;

    std::string imu_frame_id_;
    std::string insgps_frame_id_;
    std::string latlon_frame_id_;

    std::string imu_topic_;
    std::string mag_pose_2d_topic_;
    std::string latlon_topic_;
    std::string euler_angles_topic_;
    std::string magnetic_topic_;
    std::string gps_topic_;
    std::string twist_topic_;
    std::string ned_odom_topic_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr mag_pose_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_angles_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr magnetic_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ned_odom_publisher_;

    void load_parameters();
    void create_publishers();
    bool initialize_serial();
    void process_data();
    bool validate_frame_length(const FrameType frame_type, size_t frame_length);
    bool validate_crc8(uint8_t* data, size_t length, uint8_t expected_crc);
    bool validate_crc16(uint8_t* data, uint8_t frame_length, uint16_t expected_crc);
    void update_serial_number(uint8_t received_sn);
    void transform_imu_data(sensor_msgs::msg::Imu& imu_msg);
    void publish_imu_data();
    void publish_ahrs_data();
    void publish_geodetic_pos_data();
    void publish_insgps_data();
    void calculate_magnetic_yaw(double roll, double pitch, double& yaw, 
                               double magx, double magy, double magz);
};

}  // namespace FDILink

#endif  // FDILINK_AHRS_DRIVER_H    