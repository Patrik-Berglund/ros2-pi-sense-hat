#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros2_pi_sense_hat/srv/set_pixel.hpp"
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
    
    // Service to set individual pixels
    pixel_srv_ = create_service<ros2_pi_sense_hat::srv::SetPixel>(
      "/sense_hat/led_matrix/set_pixel",
      std::bind(&LEDMatrixNode::setPixelCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    // Service to update display
    update_srv_ = create_service<std_srvs::srv::Trigger>(
      "/sense_hat/led_matrix/update",
      std::bind(&LEDMatrixNode::updateCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(get_logger(), "Subscribed to /sense_hat/led_matrix/image");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/led_matrix/clear ready");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/led_matrix/set_pixel ready");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/led_matrix/update ready");
  }

private:
  void setPixelCallback(const ros2_pi_sense_hat::srv::SetPixel::Request::SharedPtr request,
                        ros2_pi_sense_hat::srv::SetPixel::Response::SharedPtr response) {
    driver_.setPixel(request->x, request->y, request->r, request->g, request->b);
    // No update() call - allows batching multiple pixels
    
    response->success = true;
    response->message = "Pixel set";
    RCLCPP_INFO(get_logger(), "Set pixel (%d,%d) to RGB(%d,%d,%d)", 
                request->x, request->y, request->r, request->g, request->b);
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
    
    // Convert from interleaved RGB8 to planar RGB5 format
    for (int y = 0; y < 8; y++) {
      for (int x = 0; x < 8; x++) {
        int src_idx = (y * 8 + x) * 3;  // RGB interleaved index
        uint8_t r = msg->data[src_idx] >> 3;     // Scale to 5-bit
        uint8_t g = msg->data[src_idx + 1] >> 3;
        uint8_t b = msg->data[src_idx + 2] >> 3;
        driver_.setPixel(x, y, r, g, b);
      }
    }
    
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
  
  void updateCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                      std_srvs::srv::Trigger::Response::SharedPtr response) {
    driver_.update();
    response->success = true;
    response->message = "Display updated";
    RCLCPP_INFO(get_logger(), "Display updated via service");
  }

  ATTiny88Driver driver_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::Service<ros2_pi_sense_hat::srv::SetPixel>::SharedPtr pixel_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr update_srv_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDMatrixNode>());
  rclcpp::shutdown();
  return 0;
}
