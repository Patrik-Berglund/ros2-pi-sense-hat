#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "ros2_pi_sense_hat/srv/read_joystick.hpp"
#include "ros2_pi_sense_hat/attiny88_driver.hpp"
#include <thread>
#include <atomic>
#include <signal.h>

std::shared_ptr<rclcpp::Node> g_node = nullptr;

void signalHandler(int signum) {
  if (g_node) {
    RCLCPP_INFO(g_node->get_logger(), "Interrupt signal (%d) received. Shutting down...", signum);
  }
  rclcpp::shutdown();
  exit(signum);
}

class JoystickNode : public rclcpp::Node {
public:
  JoystickNode() : Node("joystick_node"), driver_(this), interrupt_thread_running_(false) {
    // Initialize only joystick interrupt (GPIO23) for joystick
    driver_.initJoystickInterrupt();
    
    RCLCPP_INFO(get_logger(), "Joystick node initialized");
    
    // Publish joystick events
    joystick_pub_ = create_publisher<sensor_msgs::msg::Joy>("/sense_hat/joystick", 10);
    
    // Service to read joystick on-demand
    joystick_srv_ = create_service<ros2_pi_sense_hat::srv::ReadJoystick>(
      "/sense_hat/joystick/read",
      std::bind(&JoystickNode::readJoystickCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(get_logger(), "Publishing to /sense_hat/joystick");
    RCLCPP_INFO(get_logger(), "Service /sense_hat/joystick/read ready");
    
    // Start interrupt thread
    interrupt_thread_running_ = true;
    interrupt_thread_ = std::thread(&JoystickNode::interruptThreadFunc, this);
  }
  
  ~JoystickNode() {
    interrupt_thread_running_ = false;
    if (interrupt_thread_.joinable()) {
      interrupt_thread_.join();
    }
  }

private:
  void readJoystickCallback(const ros2_pi_sense_hat::srv::ReadJoystick::Request::SharedPtr,
                           ros2_pi_sense_hat::srv::ReadJoystick::Response::SharedPtr response) {
    response->buttons = driver_.readJoystick();
    RCLCPP_INFO(get_logger(), "Joystick read: 0x%02X", response->buttons);
  }
  
  void interruptThreadFunc() {
    uint8_t last_buttons = 0;
    
    while (interrupt_thread_running_) {
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
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joystick_pub_;
  rclcpp::Service<ros2_pi_sense_hat::srv::ReadJoystick>::SharedPtr joystick_srv_;
  
  std::thread interrupt_thread_;
  std::atomic<bool> interrupt_thread_running_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  // Set up signal handlers for graceful shutdown
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);
  
  g_node = std::make_shared<JoystickNode>();
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  return 0;
}
