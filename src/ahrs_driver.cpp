// ahrs_driver.cpp
#include "ahrs_driver.h"

namespace FDILink
{

  AhrsDriver::AhrsDriver(const rclcpp::NodeOptions &options)
      : Node("ahrs_driver", options),
        sn_lost_(0),
        current_serial_num_(0),
        first_serial_num_received_(false),
        device_type_(1)
  {
    load_parameters();
    if (!initialize_serial())
    {
      throw std::runtime_error("Failed to initialize serial port");
    }
    create_publishers();
    process_data();
  }

  AhrsDriver::~AhrsDriver()
  {
    if (serial_port_.isOpen())
    {
      serial_port_.close();
    }
  }

  void AhrsDriver::load_parameters()
  {
    // 使用更安全的参数加载方式
    declare_parameter("if_debug", false);
    get_parameter("if_debug", if_debug_);

    declare_parameter("device_type", 1);
    get_parameter("device_type", device_type_);

    declare_parameter("imu_topic", "/imu");
    get_parameter("imu_topic", imu_topic_);

    declare_parameter("imu_frame_id", "gyro_link");
    get_parameter("imu_frame_id", imu_frame_id_);

    declare_parameter("mag_pose_2d_topic", "/mag_pose_2d");
    get_parameter("mag_pose_2d_topic", mag_pose_2d_topic_);

    declare_parameter("Euler_angles_topic", "/euler_angles");
    get_parameter("Euler_angles_topic", euler_angles_topic_);

    declare_parameter("gps_topic", "/gps/fix");
    get_parameter("gps_topic", gps_topic_);

    declare_parameter("Magnetic_topic", "/magnetic");
    get_parameter("Magnetic_topic", magnetic_topic_);

    declare_parameter("twist_topic", "/system_speed");
    get_parameter("twist_topic", twist_topic_);

    declare_parameter("NED_odom_topic", "/NED_odometry");
    get_parameter("NED_odom_topic", ned_odom_topic_);

    declare_parameter("serial_port", "/dev/fdilink_ahrs");
    get_parameter("serial_port", serial_port_name_);

    declare_parameter("serial_baud", 921600);
    get_parameter("serial_baud", serial_baudrate_);

    declare_parameter("serial_timeout", 100);
    get_parameter("serial_timeout", serial_timeout_);

    RCLCPP_INFO(get_logger(), "Load parameters successfully");
  }

  bool AhrsDriver::initialize_serial()
  {
    try
    {
      serial_port_.setPort(serial_port_name_);
      serial_port_.setBaudrate(serial_baudrate_);
      serial_port_.setFlowcontrol(serial::flowcontrol_none);
      serial_port_.setParity(serial::parity_none);
      serial_port_.setStopbits(serial::stopbits_one);
      serial_port_.setBytesize(serial::eightbits);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
      serial_port_.setTimeout(time_out);
      serial_port_.open();
      RCLCPP_INFO(get_logger(), "Serial port initialized successfully");
      return true;
    }
    catch (const serial::IOException &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port: %s", e.what());
      return false;
    }
  }

  void AhrsDriver::process_data()
  {
    RCLCPP_INFO(get_logger(), "Starting data processing loop");
    while (rclcpp::ok())
    {
      if (!serial_port_.isOpen())
      {
        RCLCPP_WARN(get_logger(), "Serial port is not open");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      // IO1：帧头（1字节）
      uint8_t frame_head;
      if (serial_port_.read(&frame_head, 1) != 1 ||
          static_cast<FrameType>(frame_head) != FrameType::FRAME_HEAD)
      {
        continue;
      }

      // IO2：合并读取后续数据（帧类型+长度+序列号+CRC8+CRC16，共6字节）
      uint8_t head_buffer[6]; // 存储后续数据的缓冲区 [0:帧类型][1:帧长度][2:序列号][3:CRC8][4:CRC16高][5:CRC16低]
      if (serial_port_.read(head_buffer, sizeof(head_buffer)) != sizeof(head_buffer))
      {
        continue;
      }

      // 从缓冲区解析数据
      const FrameType frame_type = static_cast<FrameType>(head_buffer[0]); 
      const uint8_t frame_length = head_buffer[1];                         
      const uint8_t serial_num = head_buffer[2];                           
      const uint8_t crc8 = head_buffer[3];                                 

      // 合并CRC16（大端序：高位在前）
      const uint16_t crc16 = (static_cast<uint16_t>(head_buffer[4]) << 8) | head_buffer[5];

      // IO3:读取数据
      std::vector<uint8_t> data(frame_length);
      if (serial_port_.read(data.data(), frame_length) != static_cast<size_t>(frame_length))
      {
        continue;
      }

      ///TODO queue+新线程处理数据

      // 校验帧长度
      if (!validate_frame_length(frame_type, frame_length))
      {
        if (frame_type != FrameType::GROUND)
        {
          RCLCPP_WARN(get_logger(), "Frame length %02X validation failed for frame type 0x%02X", frame_length, static_cast<uint8_t>(frame_type));
          continue;
        }
      }

      // CRC8校验数据:帧头、帧类型、帧长度、序列号
      std::vector<uint8_t> crc8_check;
      crc8_check.push_back(frame_head);
      crc8_check.push_back(static_cast<uint8_t>(frame_type));
      crc8_check.push_back(frame_length);
      crc8_check.push_back(serial_num);

      // 检查CRC8
      if (!validate_crc8(crc8_check.data(), 4, crc8))
      {
        RCLCPP_WARN(get_logger(), "CRC8 validation failed");
        continue;
      }

      // 检查CRC16
      if (!validate_crc16(data.data(), frame_length, crc16))
      {
        RCLCPP_WARN(get_logger(), "CRC16 validation failed");
        continue;
      }

      // 更新序列号
      update_serial_number(serial_num);

      // 处理不同类型的帧
      switch (frame_type)
      {
      case FrameType::IMU:
        std::copy(crc8_check.begin(), crc8_check.end(), imu_frame_.read_buf.frame_header);
        std::copy(data.begin(), data.end(), imu_frame_.read_buf.read_msg);
        publish_imu_data();
        break;
      case FrameType::AHRS:
        std::copy(crc8_check.begin(), crc8_check.end(), ahrs_frame_.read_buf.frame_header);
        std::copy(data.begin(), data.end(), ahrs_frame_.read_buf.read_msg);
        publish_ahrs_data();
        break;
      case FrameType::INSGPS:
        std::copy(crc8_check.begin(), crc8_check.end(), insgps_frame_.read_buf.frame_header);
        std::copy(data.begin(), data.end(), insgps_frame_.read_buf.read_msg);
        publish_insgps_data();
        break;
      case FrameType::GEODETIC_POS:
        std::copy(crc8_check.begin(), crc8_check.end(), geodetic_pos_frame_.read_buf.frame_header);
        std::copy(data.begin(), data.end(), geodetic_pos_frame_.read_buf.read_msg);
        publish_geodetic_pos_data();
        break;
      case FrameType::GROUND:
        break;
      default:
        RCLCPP_WARN(get_logger(), "Unknown frame type: 0x%02X", static_cast<uint8_t>(frame_type));
        break;
      }
    }
  }

  bool AhrsDriver::validate_frame_length(const FrameType frame_type, size_t frame_length)
  {
    switch (frame_type)
    {
    case FrameType::IMU:
      return frame_length == IMU_FRAME_LEN;
    case FrameType::AHRS:
      return frame_length == AHRS_FRAME_LEN;
    case FrameType::INSGPS:
      return frame_length == INSGPS_FRAME_LEN;
    case FrameType::GEODETIC_POS:
      return frame_length == GEODETIC_POS_FRAME_LEN;
    case FrameType::GROUND: // 心跳包跳过
      return false;
    default:
      RCLCPP_WARN(get_logger(), "Unknown frame type for length validation: 0x%02X", static_cast<uint8_t>(frame_type));
      return false;
    }
  }

  bool AhrsDriver::validate_crc8(uint8_t *data, size_t length, uint8_t expected_crc)
  {
    uint8_t calculated_crc = CRC8_Table(data, length);
    if (calculated_crc != expected_crc)
    {
      RCLCPP_WARN(get_logger(), "CRC8 mismatch (expected: 0x%02X, calculated: 0x%02X)",
                  expected_crc, calculated_crc);
      return false;
    }
    return true;
  }

  bool AhrsDriver::validate_crc16(uint8_t *data, uint8_t length, uint16_t expected_crc)
  {
    uint16_t calculated_crc = CRC16_Table(data, length);
    if (calculated_crc != expected_crc)
    {
      RCLCPP_WARN(get_logger(), "CRC16 mismatch (expected: 0x%04X, calculated: 0x%04X)",
                  expected_crc, calculated_crc);
      return false;
    }
    return true;
  }

  void AhrsDriver::update_serial_number(uint8_t received_sn)
  {
    if (!first_serial_num_received_)
    {
      current_serial_num_ = received_sn - 1;
      first_serial_num_received_ = true;
    }

    int delta = received_sn - current_serial_num_;
    if (delta < 0)
    {
      delta += 256;
    }
    sn_lost_ += delta - 1;
    current_serial_num_ = received_sn;

    if (if_debug_)
    {
      RCLCPP_DEBUG(get_logger(), "Serial number updated: current=%d, lost=%d",
                   current_serial_num_, sn_lost_);
    }
  }

  void AhrsDriver::transform_imu_data(sensor_msgs::msg::Imu &imu_msg)
  {
    Eigen::Quaterniond q_ahrs(ahrs_frame_.frame.data.data_pack.Qw,
                              ahrs_frame_.frame.data.data_pack.Qx,
                              ahrs_frame_.frame.data.data_pack.Qy,
                              ahrs_frame_.frame.data.data_pack.Qz);
    Eigen::Quaterniond q_r =
        Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_rr =
        Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.00000, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(PI, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_out;
    if (device_type_ == 0) // 未经变换的原始数据
    {
      imu_msg.orientation.w = ahrs_frame_.frame.data.data_pack.Qw;
      imu_msg.orientation.x = ahrs_frame_.frame.data.data_pack.Qx;
      imu_msg.orientation.y = ahrs_frame_.frame.data.data_pack.Qy;
      imu_msg.orientation.z = ahrs_frame_.frame.data.data_pack.Qz;
    }
    else if (device_type_ == 1) // imu单品rclcpp标准下的坐标变换
    {
      q_out = q_r * q_ahrs * q_rr;
      imu_msg.orientation.w = q_out.w();
      imu_msg.orientation.x = q_out.x();
      imu_msg.orientation.y = q_out.y();
      imu_msg.orientation.z = q_out.z();
    }
    imu_msg.angular_velocity.x = imu_frame_.frame.data.data_pack.gyroscope_x;
    imu_msg.angular_velocity.y = -imu_frame_.frame.data.data_pack.gyroscope_y;
    imu_msg.angular_velocity.z = -imu_frame_.frame.data.data_pack.gyroscope_z;
    imu_msg.linear_acceleration.x = imu_frame_.frame.data.data_pack.accelerometer_x;
    imu_msg.linear_acceleration.y = -imu_frame_.frame.data.data_pack.accelerometer_y;
    imu_msg.linear_acceleration.z = -imu_frame_.frame.data.data_pack.accelerometer_z;
  }

  void AhrsDriver::publish_imu_data()
  {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now();
    imu_msg.header.frame_id = imu_frame_id_;
    transform_imu_data(imu_msg);
    imu_publisher_->publish(imu_msg);
  }

  void AhrsDriver::publish_ahrs_data()
  {
    geometry_msgs::msg::Pose2D pose_2d;
    pose_2d.theta = ahrs_frame_.frame.data.data_pack.Heading;
    mag_pose_publisher_->publish(pose_2d);

    geometry_msgs::msg::Vector3 euler_angles;
    euler_angles.x = ahrs_frame_.frame.data.data_pack.Roll;
    euler_angles.y = ahrs_frame_.frame.data.data_pack.Pitch;
    euler_angles.z = ahrs_frame_.frame.data.data_pack.Heading;
    euler_angles_publisher_->publish(euler_angles);

    geometry_msgs::msg::Vector3 magnetic;
    magnetic.x = imu_frame_.frame.data.data_pack.magnetometer_x;
    magnetic.y = imu_frame_.frame.data.data_pack.magnetometer_y;
    magnetic.z = imu_frame_.frame.data.data_pack.magnetometer_z;
    magnetic_publisher_->publish(magnetic);
  }

  void AhrsDriver::publish_geodetic_pos_data()
  {
    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.stamp = now();
    gps_msg.header.frame_id = "navsat_link";
    gps_msg.latitude = geodetic_pos_frame_.frame.data.data_pack.Latitude / DEG_TO_RAD;
    gps_msg.longitude = geodetic_pos_frame_.frame.data.data_pack.Longitude / DEG_TO_RAD;
    gps_msg.altitude = geodetic_pos_frame_.frame.data.data_pack.Height;
    gps_publisher_->publish(gps_msg);
  }

  void AhrsDriver::publish_insgps_data()
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now();
    odom_msg.pose.pose.position.x = insgps_frame_.frame.data.data_pack.Location_North;
    odom_msg.pose.pose.position.y = insgps_frame_.frame.data.data_pack.Location_East;
    odom_msg.pose.pose.position.z = insgps_frame_.frame.data.data_pack.Location_Down;
    odom_msg.twist.twist.linear.x = insgps_frame_.frame.data.data_pack.Velocity_North;
    odom_msg.twist.twist.linear.y = insgps_frame_.frame.data.data_pack.Velocity_East;
    odom_msg.twist.twist.linear.z = insgps_frame_.frame.data.data_pack.Velocity_Down;
    ned_odom_publisher_->publish(odom_msg);

    geometry_msgs::msg::Twist speed_msg;
    speed_msg.linear.x = insgps_frame_.frame.data.data_pack.BodyVelocity_X;
    speed_msg.linear.y = insgps_frame_.frame.data.data_pack.BodyVelocity_Y;
    speed_msg.linear.z = insgps_frame_.frame.data.data_pack.BodyVelocity_Z;
    twist_publisher_->publish(speed_msg);
  }

  void AhrsDriver::calculate_magnetic_yaw(double roll, double pitch, double &yaw,
                                          double magx, double magy, double magz)
  {
    double temp1 = magy * cos(roll) + magz * sin(roll);
    double temp2 = magx * cos(pitch) + magy * sin(pitch) * sin(roll) - magz * sin(pitch) * cos(roll);
    yaw = atan2(-temp1, temp2);
    if (yaw < 0)
    {
      yaw += 2 * PI;
    }
  }

  void AhrsDriver::create_publishers()
  {
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    mag_pose_publisher_ = create_publisher<geometry_msgs::msg::Pose2D>(mag_pose_2d_topic_, 10);
    gps_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(gps_topic_, 10);
    euler_angles_publisher_ = create_publisher<geometry_msgs::msg::Vector3>(euler_angles_topic_, 10);
    magnetic_publisher_ = create_publisher<geometry_msgs::msg::Vector3>(magnetic_topic_, 10);
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 10);
    ned_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(ned_odom_topic_, 10);
  }

} // namespace FDILink

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<FDILink::AhrsDriver>());
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ahrs_driver"), "Exception: %s", e.what());
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}