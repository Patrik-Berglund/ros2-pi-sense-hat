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
    // Publish rate
    rcl_interfaces::msg::ParameterDescriptor rate_desc;
    rate_desc.description = "Publishing rate in Hz";
    rate_desc.integer_range.resize(1);
    rate_desc.integer_range[0].from_value = 1;
    rate_desc.integer_range[0].to_value = 100;
    declare_parameter("publish_rate", 1, rate_desc);
    
    // Temperature offsets
    rcl_interfaces::msg::ParameterDescriptor temp_offset_desc;
    temp_offset_desc.description = "Temperature calibration offset in °C";
    temp_offset_desc.floating_point_range.resize(1);
    temp_offset_desc.floating_point_range[0].from_value = -50.0;
    temp_offset_desc.floating_point_range[0].to_value = 50.0;
    declare_parameter("temperature_offset_hts221", 0.0, temp_offset_desc);
    declare_parameter("temperature_offset_lps25h", 0.0, temp_offset_desc);
    
    // HTS221 ODR: 0=one-shot, 1=1Hz, 2=7Hz, 3=12.5Hz
    rcl_interfaces::msg::ParameterDescriptor hts_odr_desc;
    hts_odr_desc.description = "HTS221 ODR: 0=one-shot, 1=1Hz, 2=7Hz, 3=12.5Hz";
    hts_odr_desc.integer_range.resize(1);
    hts_odr_desc.integer_range[0].from_value = 0;
    hts_odr_desc.integer_range[0].to_value = 3;
    declare_parameter("hts221_odr", 1, hts_odr_desc);
    
    // LPS25H ODR: 0=one-shot, 1=1Hz, 2=7Hz, 3=12.5Hz, 4=25Hz
    rcl_interfaces::msg::ParameterDescriptor lps_odr_desc;
    lps_odr_desc.description = "LPS25H ODR: 0=one-shot, 1=1Hz, 2=7Hz, 3=12.5Hz, 4=25Hz";
    lps_odr_desc.integer_range.resize(1);
    lps_odr_desc.integer_range[0].from_value = 0;
    lps_odr_desc.integer_range[0].to_value = 4;
    declare_parameter("lps25h_odr", 1, lps_odr_desc);
    
    // HTS221 averaging: 0-7
    rcl_interfaces::msg::ParameterDescriptor hts_avg_desc;
    hts_avg_desc.description = "Averaging samples: 0-7 (higher = more averaging)";
    hts_avg_desc.integer_range.resize(1);
    hts_avg_desc.integer_range[0].from_value = 0;
    hts_avg_desc.integer_range[0].to_value = 7;
    declare_parameter("hts221_temp_avg", 2, hts_avg_desc);
    declare_parameter("hts221_hum_avg", 4, hts_avg_desc);
    
    // LPS25H averaging: 0-3
    rcl_interfaces::msg::ParameterDescriptor lps_avg_desc;
    lps_avg_desc.description = "Averaging: 0=8, 1=32, 2=128, 3=512 samples";
    lps_avg_desc.integer_range.resize(1);
    lps_avg_desc.integer_range[0].from_value = 0;
    lps_avg_desc.integer_range[0].to_value = 3;
    declare_parameter("lps25h_press_avg", 2, lps_avg_desc);
    
    rcl_interfaces::msg::ParameterDescriptor lps_temp_avg_desc;
    lps_temp_avg_desc.description = "Averaging: 0=8, 1=16, 2=32, 3=64 samples";
    lps_temp_avg_desc.integer_range.resize(1);
    lps_temp_avg_desc.integer_range[0].from_value = 0;
    lps_temp_avg_desc.integer_range[0].to_value = 3;
    declare_parameter("lps25h_temp_avg", 2, lps_temp_avg_desc);
    
    // LPS25H FIFO
    rcl_interfaces::msg::ParameterDescriptor fifo_desc;
    fifo_desc.description = "Enable FIFO mean mode for temporal averaging";
    declare_parameter("lps25h_fifo_mean", false, fifo_desc);
    
    rcl_interfaces::msg::ParameterDescriptor fifo_samples_desc;
    fifo_samples_desc.description = "FIFO samples to average (2-32)";
    fifo_samples_desc.integer_range.resize(1);
    fifo_samples_desc.integer_range[0].from_value = 2;
    fifo_samples_desc.integer_range[0].to_value = 32;
    declare_parameter("lps25h_fifo_samples", 16, fifo_samples_desc);
    
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
    
    hts221_.enable();
    lps25h_.enable();
    
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
