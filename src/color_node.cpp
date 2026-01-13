#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "ros2_pi_sense_hat/tcs3400_driver.hpp"

class ColorNode : public rclcpp::Node {
public:
  ColorNode() : Node("color_node") {
    rcl_interfaces::msg::ParameterDescriptor rate_desc;
    rate_desc.description = "Publishing rate in Hz";
    rate_desc.integer_range.resize(1);
    rate_desc.integer_range[0].from_value = 1;
    rate_desc.integer_range[0].to_value = 100;
    declare_parameter("publish_rate", 10, rate_desc);
    
    rcl_interfaces::msg::ParameterDescriptor atime_desc;
    atime_desc.description = "Integration time (ATIME register 0x00-0xFF, lower=longer)";
    atime_desc.integer_range.resize(1);
    atime_desc.integer_range[0].from_value = 0;
    atime_desc.integer_range[0].to_value = 255;
    declare_parameter("integration_time", 0xF6, atime_desc);
    
    rcl_interfaces::msg::ParameterDescriptor gain_desc;
    gain_desc.description = "Gain: 0=1x, 1=4x, 2=16x, 3=64x";
    gain_desc.integer_range.resize(1);
    gain_desc.integer_range[0].from_value = 0;
    gain_desc.integer_range[0].to_value = 3;
    declare_parameter("gain", 2, gain_desc);
    
    rcl_interfaces::msg::ParameterDescriptor lux_desc;
    lux_desc.description = "Lux calibration multiplier";
    lux_desc.floating_point_range.resize(1);
    lux_desc.floating_point_range[0].from_value = 0.1;
    lux_desc.floating_point_range[0].to_value = 10.0;
    declare_parameter("lux_calibration", 1.0, lux_desc);
    
    rcl_interfaces::msg::ParameterDescriptor wait_en_desc;
    wait_en_desc.description = "Enable wait time between measurements for power saving";
    declare_parameter("wait_enable", false, wait_en_desc);
    
    rcl_interfaces::msg::ParameterDescriptor wtime_desc;
    wtime_desc.description = "Wait time (WTIME register 0x00-0xFF, lower=longer)";
    wtime_desc.integer_range.resize(1);
    wtime_desc.integer_range[0].from_value = 0;
    wtime_desc.integer_range[0].to_value = 255;
    declare_parameter("wait_time", 0xFF, wtime_desc);
    
    rcl_interfaces::msg::ParameterDescriptor wlong_desc;
    wlong_desc.description = "12x multiplier for wait time (up to 8.54s)";
    declare_parameter("wait_long", false, wlong_desc);
    
    int rate = get_parameter("publish_rate").as_int();
    lux_cal_ = get_parameter("lux_calibration").as_double();
    
    if (!tcs3400_.init()) {
      RCLCPP_WARN(get_logger(), "TCS3400 color sensor not found - node will not publish data");
      RCLCPP_WARN(get_logger(), "This Sense HAT may not have the color sensor populated");
      return;
    }
    
    tcs3400_.set_integration_time(get_parameter("integration_time").as_int());
    tcs3400_.set_gain(get_parameter("gain").as_int());
    tcs3400_.set_wait_time(
      get_parameter("wait_enable").as_bool(),
      get_parameter("wait_time").as_int(),
      get_parameter("wait_long").as_bool());
    tcs3400_.enable();
    
    illuminance_pub_ = create_publisher<sensor_msgs::msg::Illuminance>(
      "sense_hat/color/illuminance", 10);
    rgb_pub_ = create_publisher<std_msgs::msg::ColorRGBA>(
      "sense_hat/color/rgb", 10);
    
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&ColorNode::timer_callback, this));
    
    RCLCPP_INFO(get_logger(), "Color sensor node started");
  }

private:
  void timer_callback() {
    uint16_t r, g, b, c;
    if (!tcs3400_.read_rgbc(r, g, b, c)) return;
    
    auto stamp = now();
    
    auto illum_msg = sensor_msgs::msg::Illuminance();
    illum_msg.header.stamp = stamp;
    illum_msg.header.frame_id = "sense_hat";
    illum_msg.illuminance = c * lux_cal_;
    illum_msg.variance = 0.0;
    illuminance_pub_->publish(illum_msg);
    
    auto rgb_msg = std_msgs::msg::ColorRGBA();
    if (c > 0) {
      rgb_msg.r = static_cast<float>(r) / c;
      rgb_msg.g = static_cast<float>(g) / c;
      rgb_msg.b = static_cast<float>(b) / c;
      rgb_msg.a = 1.0f;
    }
    rgb_pub_->publish(rgb_msg);
  }

  TCS3400Driver tcs3400_;
  double lux_cal_;
  rclcpp::Publisher<sensor_msgs::msg::Illuminance>::SharedPtr illuminance_pub_;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr rgb_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorNode>());
  rclcpp::shutdown();
  return 0;
}
