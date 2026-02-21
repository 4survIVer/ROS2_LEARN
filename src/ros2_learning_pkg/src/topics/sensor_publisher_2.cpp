/**
 * SENSOR PUBLISHER 2 - Multiple Publishers Example
 * 
 * Demonstrates:
 * - Second publisher on same topic
 * - Different data characteristics
 * - Independent node operation
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class SensorPublisher2 : public rclcpp::Node
{
public:
  SensorPublisher2() : Node("sensor_publisher_2"), sensor_id_("sensor_002")
  {
    // Create publisher for sensor data (SAME TOPIC as publisher 1)
    sensor_pub_ = this->create_publisher<std_msgs::msg::Float32>("sensor_temperature", 10);
    
    // Different publishing rate to demonstrate load
    timer_ = this->create_wall_timer(300ms, std::bind(&SensorPublisher2::timer_callback, this));
    
    // Different temperature characteristics
    random_gen_ = std::mt19937(rd_());
    distribution_ = std::normal_distribution<double>(22.0, 1.5); // Mean 22°C, std dev 1.5°C
    
    RCLCPP_INFO(this->get_logger(), "🧪 Sensor Publisher 2 started! ID: %s", sensor_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: /sensor_temperature");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32();
    
    // Generate simulated temperature data with different characteristics
    message.data = distribution_(random_gen_);
    
    // Publish the sensor data
    sensor_pub_->publish(message);
    
    RCLCPP_INFO(this->get_logger(), "🌡️  [%s] Temperature: %.2f°C", 
                sensor_id_.c_str(), message.data);
  }
  
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sensor_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string sensor_id_;
  
  // For random number generation
  std::random_device rd_;
  std::mt19937 random_gen_;
  std::normal_distribution<double> distribution_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorPublisher2>());
  rclcpp::shutdown();
  return 0;
}
