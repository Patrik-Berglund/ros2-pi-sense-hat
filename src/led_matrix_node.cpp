#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros2_pi_sense_hat/attiny88_driver.hpp"

class LEDMatrixNode : public rclcpp::Node {
public:
  LEDMatrixNode() : Node("led_matrix_node") {
    if (!driver_.init()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize ATTINY88");
      return;
    }
    
    driver_.clear();
    driver_.update();
    RCLCPP_INFO(get_logger(), "LED matrix initialized and cleared");
    
    // Subscribe to 8x8 RGB image (sensor_msgs/Image)
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/sense_hat/led_matrix/image", 10,
      std::bind(&LEDMatrixNode::imageCallback, this, std::placeholders::_1));
    
    // Service to clear display
    clear_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/led_matrix/clear",
      std::bind(&LEDMatrixNode::clearCallback, this, 
                std::placeholders::_1, std::placeholders::_2));
    
    // Declare parameters for single pixel control
    declare_parameter("pixel_x", 0);
    declare_parameter("pixel_y", 0);
    declare_parameter("pixel_r", 0);
    declare_parameter("pixel_g", 0);
    declare_parameter("pixel_b", 0);
    
    // Timer to check for parameter updates
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&LEDMatrixNode::timerCallback, this));
    
    RCLCPP_INFO(get_logger(), "Subscribed to /sense_hat/led_matrix/image");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/led_matrix/clear ready");
    RCLCPP_INFO(get_logger(), "Use 'ros2 param set' to control individual pixels");
  }

private:
  void timerCallback() {
    int x = get_parameter("pixel_x").as_int();
    int y = get_parameter("pixel_y").as_int();
    int r = get_parameter("pixel_r").as_int();
    int g = get_parameter("pixel_g").as_int();
    int b = get_parameter("pixel_b").as_int();
    
    if (x != last_x_ || y != last_y_ || r != last_r_ || g != last_g_ || b != last_b_) {
      driver_.setPixel(x, y, r, g, b);
      driver_.update();
      RCLCPP_INFO(get_logger(), "Set pixel (%d,%d) to RGB(%d,%d,%d)", x, y, r, g, b);
      last_x_ = x; last_y_ = y; last_r_ = r; last_g_ = g; last_b_ = b;
    }
  }
  
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Expect 8x8 RGB8 image (192 bytes)
    if (msg->width != 8 || msg->height != 8) {
      RCLCPP_WARN(get_logger(), "Image must be 8x8, got %dx%d", msg->width, msg->height);
      return;
    }
    
    if (msg->encoding != "rgb8") {
      RCLCPP_WARN(get_logger(), "Image encoding must be rgb8, got %s", msg->encoding.c_str());
      return;
    }
    
    if (msg->data.size() != 192) {
      RCLCPP_WARN(get_logger(), "Expected 192 bytes, got %zu", msg->data.size());
      return;
    }
    
    // Convert RGB8 (0-255) to RGB5 (0-31)
    uint8_t rgb5_data[192];
    for (size_t i = 0; i < 192; i++) {
      rgb5_data[i] = msg->data[i] >> 3;  // Scale 8-bit to 5-bit
    }
    
    driver_.setAll(rgb5_data, 192);
    driver_.update();
  }
  
  void clearCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                     std_srvs::srv::Trigger::Response::SharedPtr response) {
    driver_.clear();
    driver_.update();
    response->success = true;
    response->message = "Display cleared";
    RCLCPP_INFO(get_logger(), "Display cleared via service");
  }

  ATTiny88Driver driver_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  int last_x_ = -1, last_y_ = -1, last_r_ = -1, last_g_ = -1, last_b_ = -1;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDMatrixNode>());
  rclcpp::shutdown();
  return 0;
}
