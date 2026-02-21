/**
 * SENSOR PUBLISHER 1 - Multiple Publishers Example
 * 
 * Demonstrates:
 * - Multiple publishers on same topic
 * - Simulated sensor data
 * - Unique node identification
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

class SensorPublisher1 : public rclcpp::Node
{
public:
  SensorPublisher1() : Node("sensor_publisher_1"), sensor_id_("sensor_001")
  {
    // Create publisher for sensor data
    sensor_pub_ = this->create_publisher<std_msgs::msg::Float32>("sensor_temperature", 10);
    
    // Timer for publishing sensor data
    timer_ = this->create_wall_timer(500ms, std::bind(&SensorPublisher1::timer_callback, this));
    
    // Initialize random number generator for sensor noise
    random_gen_ = std::mt19937(rd_());
    distribution_ = std::normal_distribution<double>(25.0, 2.0); // Mean 25°C, std dev 2°C
    
    RCLCPP_INFO(this->get_logger(), "🧪 Sensor Publisher 1 started! ID: %s", sensor_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing to: /sensor_temperature");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Float32();
    
    // Generate simulated temperature data with some noise
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
  rclcpp::spin(std::make_shared<SensorPublisher1>());
  rclcpp::shutdown();
  return 0;
}
