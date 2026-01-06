#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros2_pi_sense_hat/lsm9ds1_driver.hpp"

class IMUNode : public rclcpp::Node {
public:
  IMUNode() : Node("imu_node"), driver_(this) {
    // Declare parameters
    declare_parameter("publish_rate", 10);
    declare_parameter("accel_range", 2);
    declare_parameter("gyro_range", 245);
    declare_parameter("mag_range", 4);
    declare_parameter("frame_id", "imu_link");
    declare_parameter("enable_magnetometer", true);
    
    // Get parameters
    int publish_rate = get_parameter("publish_rate").as_int();
    int accel_range = get_parameter("accel_range").as_int();
    int gyro_range = get_parameter("gyro_range").as_int();
    int mag_range = get_parameter("mag_range").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    enable_mag_ = get_parameter("enable_magnetometer").as_bool();

    // Initialize driver
    if (!driver_.init()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize LSM9DS1");
      return;
    }
    
    // Configure ranges
    driver_.setAccelRange(accel_range);
    driver_.setGyroRange(gyro_range);
    driver_.setMagRange(mag_range);

    // Create publishers
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/sense_hat/imu/data_raw", 10);
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("/sense_hat/temperature/imu", 10);
    if (enable_mag_) {
      mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/sense_hat/imu/mag", 10);
    }

    // Create service
    calibrate_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/imu/calibrate",
      std::bind(&IMUNode::calibrateCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create timer
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / publish_rate),
      std::bind(&IMUNode::publishData, this));

    RCLCPP_INFO(get_logger(), "IMU node initialized - publishing at %d Hz", publish_rate);
  }

private:
  void publishData() {
    IMUData data;
    if (!driver_.readAllSensors(data)) {
      RCLCPP_WARN(get_logger(), "Failed to read IMU data");
      return;
    }

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
    response->success = true;
    response->message = "Calibration not implemented - using factory calibration";
  }

  LSM9DS1Driver driver_;
  std::string frame_id_;
  bool enable_mag_;
  
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUNode>());
  rclcpp::shutdown();
  return 0;
}
