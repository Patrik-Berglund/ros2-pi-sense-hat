#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ros2_pi_sense_hat/srv/set_pixel.hpp"
#include "ros2_pi_sense_hat/srv/read_joystick.hpp"
#include "ros2_pi_sense_hat/attiny88_driver.hpp"
#include <thread>
#include <atomic>

class LEDMatrixNode : public rclcpp::Node {
public:
  LEDMatrixNode() : Node("led_matrix_node"), joystick_thread_running_(false) {
    if (!driver_.init()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize ATTINY88");
      return;
    }
    
    driver_.clear();
    RCLCPP_INFO(get_logger(), "LED matrix initialized and cleared");
    
    // Subscribe to 8x8 RGB image (sensor_msgs/Image)
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/sense_hat/led_matrix/image", 10,
      std::bind(&LEDMatrixNode::imageCallback, this, std::placeholders::_1));
    
    // Publish joystick events
    joystick_pub_ = create_publisher<sensor_msgs::msg::Joy>("/sense_hat/joystick", 10);
    
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
    
    // Service to read joystick
    joystick_srv_ = create_service<ros2_pi_sense_hat::srv::ReadJoystick>(
      "/sense_hat/joystick/read",
      std::bind(&LEDMatrixNode::readJoystickCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(get_logger(), "Subscribed to /sense_hat/led_matrix/image");
    RCLCPP_INFO(get_logger(), "Publishing to /sense_hat/joystick");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/led_matrix/clear ready");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/led_matrix/set_pixel ready");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/joystick/read ready");
    
    // Start joystick interrupt thread
    joystick_thread_running_ = true;
    joystick_thread_ = std::thread(&LEDMatrixNode::joystickThreadFunc, this);
  }
  
  ~LEDMatrixNode() {
    joystick_thread_running_ = false;
    if (joystick_thread_.joinable()) {
      joystick_thread_.join();
    }
  }

private:
  void setPixelCallback(const ros2_pi_sense_hat::srv::SetPixel::Request::SharedPtr request,
                        ros2_pi_sense_hat::srv::SetPixel::Response::SharedPtr response) {
    driver_.setPixel(request->x, request->y, request->r, request->g, request->b);
    
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
    std::vector<uint8_t> planar_data(192);
    for (int y = 0; y < 8; y++) {
      for (int x = 0; x < 8; x++) {
        int src_idx = (y * 8 + x) * 3;  // RGB interleaved index
        int row_offset = y * 24;
        planar_data[row_offset + x] = msg->data[src_idx] >> 3;         // R
        planar_data[row_offset + 8 + x] = msg->data[src_idx + 1] >> 3; // G
        planar_data[row_offset + 16 + x] = msg->data[src_idx + 2] >> 3; // B
      }
    }
    
    driver_.setAll(planar_data.data(), 192);
  }
  
  void clearCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                     std_srvs::srv::Trigger::Response::SharedPtr response) {
    driver_.clear();
    response->success = true;
    response->message = "Display cleared";
    RCLCPP_INFO(get_logger(), "Display cleared via service");
  }
  
  void readJoystickCallback(const ros2_pi_sense_hat::srv::ReadJoystick::Request::SharedPtr,
                           ros2_pi_sense_hat::srv::ReadJoystick::Response::SharedPtr response) {
    response->buttons = driver_.readJoystick();
    RCLCPP_INFO(get_logger(), "Joystick read: 0x%02X", response->buttons);
  }
  
  void joystickThreadFunc() {
    uint8_t last_buttons = 0;
    
    while (joystick_thread_running_) {
      // Wait for GPIO23 interrupt (button state change)
      if (driver_.waitForJoystickEvent(100)) {
        uint8_t buttons = driver_.readJoystick();
        
        if (buttons != last_buttons) {
          // Create Joy message
          auto joy_msg = sensor_msgs::msg::Joy();
          joy_msg.header.stamp = this->get_clock()->now();
          joy_msg.header.frame_id = "sense_hat_joystick";
          
          // Convert 5-bit button state to individual buttons
          joy_msg.buttons.resize(5);
          for (int i = 0; i < 5; i++) {
            joy_msg.buttons[i] = (buttons >> i) & 1;
          }
          
          // Publish event
          joystick_pub_->publish(joy_msg);
          RCLCPP_INFO(get_logger(), "Joystick event: 0x%02X", buttons);
          
          last_buttons = buttons;
        }
      }
    }
  }
  
  ATTiny88Driver driver_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joystick_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_srv_;
  rclcpp::Service<ros2_pi_sense_hat::srv::SetPixel>::SharedPtr pixel_srv_;
  rclcpp::Service<ros2_pi_sense_hat::srv::ReadJoystick>::SharedPtr joystick_srv_;
  
  std::thread joystick_thread_;
  std::atomic<bool> joystick_thread_running_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDMatrixNode>());
  rclcpp::shutdown();
  return 0;
}
