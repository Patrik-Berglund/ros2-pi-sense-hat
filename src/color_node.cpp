#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "ros2_pi_sense_hat/tcs3400_driver.hpp"

class ColorNode : public rclcpp::Node {
public:
  ColorNode() : Node("color_node") {
    declare_parameter("publish_rate", 10);
    declare_parameter("integration_time", 0xF6);
    declare_parameter("gain", 2);
    declare_parameter("lux_calibration", 1.0);
    declare_parameter("wait_enable", false);
    declare_parameter("wait_time", 0xFF);
    declare_parameter("wait_long", false);
    
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
