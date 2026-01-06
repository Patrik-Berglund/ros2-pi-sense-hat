#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_pi_sense_hat/action/i2_c_transaction.hpp"
#include "ros2_pi_sense_hat/i2c_device.hpp"
#include <unordered_map>
#include <memory>

class I2CBridgeNode : public rclcpp::Node {
public:
  using I2CTransaction = ros2_pi_sense_hat::action::I2CTransaction;
  using GoalHandleI2C = rclcpp_action::ServerGoalHandle<I2CTransaction>;

  I2CBridgeNode() : Node("i2c_bridge_node") {
    RCLCPP_INFO(get_logger(), "Starting I2C Bridge Node...");
    
    using namespace std::placeholders;
    
    try {
      action_server_ = rclcpp_action::create_server<I2CTransaction>(
        this,
        "i2c_transaction",
        std::bind(&I2CBridgeNode::handle_goal, this, _1, _2),
        std::bind(&I2CBridgeNode::handle_cancel, this, _1),
        std::bind(&I2CBridgeNode::handle_accepted, this, _1));

      RCLCPP_INFO(get_logger(), "I2C Bridge Node started successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to create action server: %s", e.what());
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const I2CTransaction::Goal> goal) {
    
    if (goal->device_address == 0) {
      RCLCPP_WARN(get_logger(), "Invalid device address: 0x%02X", goal->device_address);
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleI2C>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleI2C> goal_handle) {
    std::thread{std::bind(&I2CBridgeNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleI2C> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<I2CTransaction::Result>();
    
    // Get or create I2C device
    auto device = get_device(goal->device_address);
    if (!device || !device->isOpen()) {
      result->success = false;
      result->error_message = "Failed to open I2C device";
      goal_handle->succeed(result);
      return;
    }

    // Perform I2C transaction
    bool success = true;
    
    // Write operation
    if (!goal->write_data.empty()) {
      if (goal->write_data.size() == 1) {
        // Single register write
        success = device->writeReg(goal->register_address, goal->write_data[0]);
      } else {
        // Multi-byte write (register + data)
        std::vector<uint8_t> write_buffer;
        write_buffer.push_back(goal->register_address);
        write_buffer.insert(write_buffer.end(), goal->write_data.begin(), goal->write_data.end());
        success = device->write(write_buffer.data(), write_buffer.size());
      }
    }
    
    // Read operation
    if (success && goal->read_length > 0) {
      result->read_data.resize(goal->read_length);
      if (goal->write_data.empty()) {
        // Read with register address
        success = device->readMultiReg(goal->register_address, result->read_data.data(), goal->read_length);
      } else {
        // Direct read (register already written)
        success = device->read(result->read_data.data(), goal->read_length);
      }
    }

    result->success = success;
    if (!success) {
      result->error_message = "I2C transaction failed";
    }
    
    goal_handle->succeed(result);
  }

  std::shared_ptr<I2CDevice> get_device(uint8_t address) {
    auto it = devices_.find(address);
    if (it != devices_.end()) {
      return it->second;
    }
    
    // Create new device
    auto device = std::make_shared<I2CDevice>("/dev/i2c-1", address);
    if (device->open()) {
      devices_[address] = device;
      RCLCPP_INFO(get_logger(), "Opened I2C device at 0x%02X", address);
      return device;
    }
    
    RCLCPP_ERROR(get_logger(), "Failed to open I2C device at 0x%02X", address);
    return nullptr;
  }

  rclcpp_action::Server<I2CTransaction>::SharedPtr action_server_;
  std::unordered_map<uint8_t, std::shared_ptr<I2CDevice>> devices_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<I2CBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
