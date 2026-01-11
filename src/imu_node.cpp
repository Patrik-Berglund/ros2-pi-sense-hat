#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <signal.h>
#include "ros2_pi_sense_hat/lsm9ds1_driver.hpp"
#include "ros2_pi_sense_hat/imu_calibration.hpp"

// Global pointer for signal handler
static std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signalHandler(int signum) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Interrupt signal (%d) received. Shutting down...", signum);
    rclcpp::shutdown();
  }
}

class IMUNode : public rclcpp::Node {
public:
  IMUNode() : Node("imu_node") {
    // Declare parameters with validation
    declare_parameter("publish_rate", 10);
    
    // IMU parameters (shared ODR for accel+gyro)
    declare_parameter("imu_odr", 119);
    declare_parameter("accel_range", 2);
    declare_parameter("gyro_range", 245);
    declare_parameter("accel_bandwidth_auto", true);
    declare_parameter("accel_bandwidth", 1);
    
    // Magnetometer parameters (independent)
    declare_parameter("mag_odr", 10);
    declare_parameter("mag_range", 4);
    declare_parameter("mag_performance_mode", 2);
    declare_parameter("mag_temp_compensation", true);
    
    // Standard parameters
    declare_parameter("frame_id", "imu_link");
    declare_parameter("enable_magnetometer", true);
    
    // Get and validate parameters
    int publish_rate = get_parameter("publish_rate").as_int();
    int imu_odr = get_parameter("imu_odr").as_int();
    int accel_range = get_parameter("accel_range").as_int();
    int gyro_range = get_parameter("gyro_range").as_int();
    bool accel_bw_auto = get_parameter("accel_bandwidth_auto").as_bool();
    int accel_bw = get_parameter("accel_bandwidth").as_int();
    
    int mag_odr = get_parameter("mag_odr").as_int();
    int mag_range = get_parameter("mag_range").as_int();
    int mag_perf_mode = get_parameter("mag_performance_mode").as_int();
    bool mag_temp_comp = get_parameter("mag_temp_compensation").as_bool();
    
    frame_id_ = get_parameter("frame_id").as_string();
    enable_mag_ = get_parameter("enable_magnetometer").as_bool();

    // Validate parameters
    if (!validateParameters(imu_odr, accel_range, gyro_range, accel_bw_auto, accel_bw,
                           mag_odr, mag_range, mag_perf_mode)) {
      RCLCPP_ERROR(get_logger(), "Invalid parameter configuration");
      return;
    }

    // Initialize driver
    if (!driver_.init()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize LSM9DS1");
      return;
    }
    
    // Configure IMU with parameters
    if (!driver_.setIMUConfig(imu_odr, accel_range, gyro_range, accel_bw_auto, accel_bw)) {
      RCLCPP_ERROR(get_logger(), "Failed to configure IMU");
      return;
    }
    
    // Configure magnetometer if enabled
    if (enable_mag_) {
      if (!driver_.setMagConfig(mag_odr, mag_range, mag_perf_mode, mag_temp_comp)) {
        RCLCPP_WARN(get_logger(), "Failed to configure magnetometer - continuing without it");
        enable_mag_ = false;
      }
    }

    // Create publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/imu/temperature", 10);
    if (enable_mag_) {
      mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
    }

    // Create services
    calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/imu/calibrate",
      std::bind(&IMUNode::calibrateCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    calibrate_gyro_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/imu/calibrate_gyro",
      std::bind(&IMUNode::calibrateGyroCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    calibrate_accel_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/imu/calibrate_accel",
      std::bind(&IMUNode::calibrateAccelCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    calibrate_mag_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/imu/calibrate_mag",
      std::bind(&IMUNode::calibrateMagCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    save_calibration_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/imu/save_calibration",
      std::bind(&IMUNode::saveCalibrationCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate),
      std::bind(&IMUNode::publishData, this));

    RCLCPP_INFO(get_logger(), "IMU node initialized:");
    RCLCPP_INFO(get_logger(), "  IMU ODR: %d Hz, Accel: ±%dg, Gyro: ±%d dps", 
                imu_odr, accel_range, gyro_range);
    RCLCPP_INFO(get_logger(), "  Accel BW: %s (%d)", accel_bw_auto ? "auto" : "manual", accel_bw);
    if (enable_mag_) {
      RCLCPP_INFO(get_logger(), "  Mag ODR: %d Hz, Range: ±%d gauss, Mode: %d", 
                  mag_odr, mag_range, mag_perf_mode);
    }
    RCLCPP_INFO(get_logger(), "  Publishing at %d Hz", publish_rate);
    
    // Load existing calibration if available
    std::string cal_filename = "imu_calibration.yaml";
    if (calibration_.loadCalibration(cal_filename)) {
      RCLCPP_INFO(get_logger(), "Loaded calibration from %s", cal_filename.c_str());
      
      // Display loaded calibration values
      const auto& cal_data = calibration_.getCalibrationData();
      
      if (cal_data.gyro_calibrated) {
        RCLCPP_INFO(get_logger(), "  Gyro bias: [%.6f, %.6f, %.6f] rad/s", 
                    cal_data.gyro_bias_x, cal_data.gyro_bias_y, cal_data.gyro_bias_z);
      } else {
        RCLCPP_INFO(get_logger(), "  Gyro: NOT calibrated");
      }
      
      if (cal_data.accel_calibrated) {
        RCLCPP_INFO(get_logger(), "  Accel offset: [%.6f, %.6f, %.6f] m/s²", 
                    cal_data.accel_offset_x, cal_data.accel_offset_y, cal_data.accel_offset_z);
        RCLCPP_INFO(get_logger(), "  Accel scale: [%.6f, %.6f, %.6f]", 
                    cal_data.accel_scale_x, cal_data.accel_scale_y, cal_data.accel_scale_z);
      } else {
        RCLCPP_INFO(get_logger(), "  Accel: NOT calibrated");
      }
      
      if (cal_data.mag_calibrated) {
        RCLCPP_INFO(get_logger(), "  Mag offset: [%.6f, %.6f, %.6f] Tesla", 
                    cal_data.mag_offset_x, cal_data.mag_offset_y, cal_data.mag_offset_z);
      } else {
        RCLCPP_INFO(get_logger(), "  Mag: NOT calibrated");
      }
      
    } else {
      RCLCPP_INFO(get_logger(), "No calibration file found, using factory defaults");
    }
  }

private:
  bool validateParameters(int imu_odr, int accel_range, int gyro_range, 
                         bool accel_bw_auto, int accel_bw,
                         int mag_odr, int mag_range, int mag_perf_mode) {
    // Validate IMU ODR
    if (imu_odr != 0 && imu_odr != 14 && imu_odr != 15 && imu_odr != 59 && 
        imu_odr != 60 && imu_odr != 119 && imu_odr != 238 && 
        imu_odr != 476 && imu_odr != 952) {
      RCLCPP_ERROR(get_logger(), "Invalid imu_odr: %d. Valid: 0,14,15,59,60,119,238,476,952", imu_odr);
      return false;
    }
    
    // Validate accelerometer range
    if (accel_range != 2 && accel_range != 4 && accel_range != 8 && accel_range != 16) {
      RCLCPP_ERROR(get_logger(), "Invalid accel_range: %d. Valid: 2,4,8,16", accel_range);
      return false;
    }
    
    // Validate gyroscope range
    if (gyro_range != 245 && gyro_range != 500 && gyro_range != 2000) {
      RCLCPP_ERROR(get_logger(), "Invalid gyro_range: %d. Valid: 245,500,2000", gyro_range);
      return false;
    }
    
    // Validate accelerometer bandwidth
    if (!accel_bw_auto && (accel_bw < 0 || accel_bw > 3)) {
      RCLCPP_ERROR(get_logger(), "Invalid accel_bandwidth: %d. Valid: 0-3", accel_bw);
      return false;
    }
    
    // Validate magnetometer ODR
    if (mag_odr < 0 || mag_odr > 80) {
      RCLCPP_ERROR(get_logger(), "Invalid mag_odr: %d. Valid: 0-80", mag_odr);
      return false;
    }
    
    // Validate magnetometer range
    if (mag_range != 4 && mag_range != 8 && mag_range != 12 && mag_range != 16) {
      RCLCPP_ERROR(get_logger(), "Invalid mag_range: %d. Valid: 4,8,12,16", mag_range);
      return false;
    }
    
    // Validate magnetometer performance mode
    if (mag_perf_mode < 0 || mag_perf_mode > 3) {
      RCLCPP_ERROR(get_logger(), "Invalid mag_performance_mode: %d. Valid: 0-3", mag_perf_mode);
      return false;
    }
    
    return true;
  }

  void publishData() {
    IMUData data;
    if (!driver_.readAllSensors(data)) {
      RCLCPP_WARN(get_logger(), "Failed to read IMU data");
      return;
    }

    // Apply calibration to sensor data
    calibration_.correctIMUData(data);

    auto stamp = now();

    // Publish IMU message (accel + gyro)
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = stamp;
    imu_msg.header.frame_id = frame_id_;
    
    imu_msg.linear_acceleration.x = data.accel_x;
    imu_msg.linear_acceleration.y = data.accel_y;
    imu_msg.linear_acceleration.z = data.accel_z;
    
    imu_msg.angular_velocity.x = data.gyro_x;
    imu_msg.angular_velocity.y = data.gyro_y;
    imu_msg.angular_velocity.z = data.gyro_z;
    
    // No orientation estimate - set covariance[0] = -1
    for (int i = 0; i < 9; i++) {
      imu_msg.linear_acceleration_covariance[i] = (i == 0) ? 0.01 : 0.0;
      imu_msg.angular_velocity_covariance[i] = (i == 0) ? 0.01 : 0.0;
      imu_msg.orientation_covariance[i] = (i == 0) ? -1.0 : 0.0;
    }
    
    imu_pub_->publish(imu_msg);

    // Publish temperature
    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = stamp;
    temp_msg.header.frame_id = frame_id_;
    temp_msg.temperature = data.temperature;
    temp_msg.variance = 1.0;
    temp_pub_->publish(temp_msg);

    // Publish magnetometer
    if (enable_mag_) {
      auto mag_msg = sensor_msgs::msg::MagneticField();
      mag_msg.header.stamp = stamp;
      mag_msg.header.frame_id = frame_id_;
      mag_msg.magnetic_field.x = data.mag_x;
      mag_msg.magnetic_field.y = data.mag_y;
      mag_msg.magnetic_field.z = data.mag_z;
      for (int i = 0; i < 9; i++) {
        mag_msg.magnetic_field_covariance[i] = (i == 0) ? 0.01 : 0.0;
      }
      mag_pub_->publish(mag_msg);
    }
  }

  void calibrateCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = false;
    response->message = "Use Python calibration service: python3 scripts/calibrate_imu.py";
  }

  void calibrateGyroCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = false;
    response->message = "Use Python calibration service: python3 scripts/calibrate_imu.py gyro";
  }

  void calibrateAccelCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = false;
    response->message = "Use Python calibration service: python3 scripts/calibrate_imu.py accel";
  }

  void calibrateMagCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    response->success = false;
    response->message = "Use Python calibration service: python3 scripts/calibrate_imu.py mag";
  }

  void saveCalibrationCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::string filename = "imu_calibration.yaml";
    bool success = calibration_.saveCalibration(filename);
    response->success = success;
    response->message = success ? "Calibration data saved to " + filename : "Failed to save calibration data";
  }

  LSM9DS1Driver driver_;
  IMUCalibration calibration_;
  std::string frame_id_;
  bool enable_mag_;
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_gyro_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_accel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_mag_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_calibration_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<IMUNode>();
  g_node = node;
  
  // Set up signal handlers for graceful shutdown
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
