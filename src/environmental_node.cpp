#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "ros2_pi_sense_hat/hts221_driver.hpp"
#include "ros2_pi_sense_hat/lps25h_driver.hpp"

class EnvironmentalNode : public rclcpp::Node {
public:
  EnvironmentalNode() : Node("environmental_node") {
    declare_parameter("publish_rate", 1);
    declare_parameter("temperature_offset_hts221", 0.0);
    declare_parameter("temperature_offset_lps25h", 0.0);
    declare_parameter("hts221_odr", 1);
    declare_parameter("lps25h_odr", 1);
    declare_parameter("hts221_temp_avg", 2);
    declare_parameter("hts221_hum_avg", 4);
    declare_parameter("lps25h_press_avg", 2);
    declare_parameter("lps25h_temp_avg", 2);
    declare_parameter("lps25h_fifo_mean", false);
    declare_parameter("lps25h_fifo_samples", 16);
    
    int rate = get_parameter("publish_rate").as_int();
    temp_offset_hts_ = get_parameter("temperature_offset_hts221").as_double();
    temp_offset_lps_ = get_parameter("temperature_offset_lps25h").as_double();
    
    if (!hts221_.init()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize HTS221");
      return;
    }
    
    if (!lps25h_.init()) {
      RCLCPP_ERROR(get_logger(), "Failed to initialize LPS25H");
      return;
    }
    
    hts221_.set_odr(get_parameter("hts221_odr").as_int());
    lps25h_.set_odr(get_parameter("lps25h_odr").as_int());
    hts221_.set_avg_samples(
      get_parameter("hts221_temp_avg").as_int(),
      get_parameter("hts221_hum_avg").as_int());
    lps25h_.set_avg_samples(
      get_parameter("lps25h_press_avg").as_int(),
      get_parameter("lps25h_temp_avg").as_int());
    lps25h_.set_fifo_mean(
      get_parameter("lps25h_fifo_mean").as_bool(),
      get_parameter("lps25h_fifo_samples").as_int());
    
    temp_hts_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      "sense_hat/temperature/humidity_sensor", 10);
    temp_lps_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      "sense_hat/temperature/pressure_sensor", 10);
    humidity_pub_ = create_publisher<sensor_msgs::msg::RelativeHumidity>(
      "sense_hat/humidity", 10);
    pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
      "sense_hat/pressure", 10);
    
    heater_service_ = create_service<std_srvs::srv::SetBool>(
      "sense_hat/set_heater",
      std::bind(&EnvironmentalNode::heater_callback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate),
      std::bind(&EnvironmentalNode::timer_callback, this));
    
    RCLCPP_INFO(get_logger(), "Environmental node started");
  }

private:
  void timer_callback() {
    auto stamp = now();
    
    float temp_hts, humidity;
    if (hts221_.read_temperature(temp_hts) && hts221_.read_humidity(humidity)) {
      auto temp_msg = sensor_msgs::msg::Temperature();
      temp_msg.header.stamp = stamp;
      temp_msg.header.frame_id = "sense_hat";
      temp_msg.temperature = temp_hts + temp_offset_hts_;
      temp_msg.variance = 0.0;
      temp_hts_pub_->publish(temp_msg);
      
      auto hum_msg = sensor_msgs::msg::RelativeHumidity();
      hum_msg.header.stamp = stamp;
      hum_msg.header.frame_id = "sense_hat";
      hum_msg.relative_humidity = humidity / 100.0;
      hum_msg.variance = 0.0;
      humidity_pub_->publish(hum_msg);
    }
    
    float temp_lps, pressure;
    if (lps25h_.read_temperature(temp_lps) && lps25h_.read_pressure(pressure)) {
      auto temp_msg = sensor_msgs::msg::Temperature();
      temp_msg.header.stamp = stamp;
      temp_msg.header.frame_id = "sense_hat";
      temp_msg.temperature = temp_lps + temp_offset_lps_;
      temp_msg.variance = 0.0;
      temp_lps_pub_->publish(temp_msg);
      
      auto press_msg = sensor_msgs::msg::FluidPressure();
      press_msg.header.stamp = stamp;
      press_msg.header.frame_id = "sense_hat";
      press_msg.fluid_pressure = pressure * 100.0;  // hPa to Pa
      press_msg.variance = 0.0;
      pressure_pub_->publish(press_msg);
    }
  }
  
  void heater_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    hts221_.set_heater(request->data);
    response->success = true;
    response->message = request->data ? "Heater enabled" : "Heater disabled";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  HTS221Driver hts221_;
  LPS25HDriver lps25h_;
  double temp_offset_hts_, temp_offset_lps_;
  
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_hts_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_lps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr heater_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EnvironmentalNode>());
  rclcpp::shutdown();
  return 0;
}
