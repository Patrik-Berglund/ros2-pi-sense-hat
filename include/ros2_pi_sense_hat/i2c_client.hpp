#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_pi_sense_hat/action/i2_c_transaction.hpp"
#include <future>
#include <cstring>

class I2CClient {
public:
  using I2CTransaction = ros2_pi_sense_hat::action::I2CTransaction;
  using ActionClient = rclcpp_action::Client<I2CTransaction>;

  I2CClient(rclcpp::Node* node, uint8_t device_address) 
    : node_(node), device_address_(device_address) {
    client_ = rclcpp_action::create_client<I2CTransaction>(node_, "i2c_transaction");
  }

  bool writeReg(uint8_t reg, uint8_t value) {
    auto goal = I2CTransaction::Goal();
    goal.device_address = device_address_;
    goal.register_address = reg;
    goal.write_data = {value};
    goal.read_length = 0;
    
    return send_goal_sync(goal);
  }

  bool readReg(uint8_t reg, uint8_t& value) {
    auto goal = I2CTransaction::Goal();
    goal.device_address = device_address_;
    goal.register_address = reg;
    goal.read_length = 1;
    
    auto result = send_goal_sync_with_result(goal);
    if (result && result->success && !result->read_data.empty()) {
      value = result->read_data[0];
      return true;
    }
    return false;
  }

  bool readMultiReg(uint8_t reg, uint8_t* data, size_t length) {
    auto goal = I2CTransaction::Goal();
    goal.device_address = device_address_;
    goal.register_address = reg;
    goal.read_length = length;
    
    auto result = send_goal_sync_with_result(goal);
    if (result && result->success && result->read_data.size() == length) {
      std::memcpy(data, result->read_data.data(), length);
      return true;
    }
    return false;
  }

  bool write(const uint8_t* data, size_t length) {
    if (length == 0) return false;
    
    auto goal = I2CTransaction::Goal();
    goal.device_address = device_address_;
    goal.register_address = data[0];  // First byte is register
    goal.write_data.assign(data + 1, data + length);  // Rest is data
    goal.read_length = 0;
    
    return send_goal_sync(goal);
  }

private:
  bool send_goal_sync(const I2CTransaction::Goal& goal) {
    auto result = send_goal_sync_with_result(goal);
    return result && result->success;
  }

  std::shared_ptr<I2CTransaction::Result> send_goal_sync_with_result(const I2CTransaction::Goal& goal) {
    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(node_->get_logger(), "I2C action server not available after 5 seconds");
      return nullptr;
    }

    auto promise = std::make_shared<std::promise<I2CTransaction::Result::SharedPtr>>();
    auto future = promise->get_future();

    auto send_goal_options = rclcpp_action::Client<I2CTransaction>::SendGoalOptions();
    send_goal_options.result_callback = [promise](const auto& result) {
      promise->set_value(result.result);
    };

    client_->async_send_goal(goal, send_goal_options);
    
    if (future.wait_for(std::chrono::milliseconds(1000)) == std::future_status::ready) {
      return future.get();
    }
    
    RCLCPP_ERROR(node_->get_logger(), "I2C transaction timeout");
    return nullptr;
  }

  rclcpp::Node* node_;
  uint8_t device_address_;
  ActionClient::SharedPtr client_;
};
