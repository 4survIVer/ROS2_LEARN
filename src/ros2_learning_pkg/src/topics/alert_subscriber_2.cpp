/**
 * ALERT SUBSCRIBER 2 - Multiple Subscribers Example
 * 
 * Demonstrates:
 * - Different subscriber with different purpose
 * - Conditional alert system
 * - Independent message processing
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class AlertSubscriber2 : public rclcpp::Node
{
public:
  AlertSubscriber2() : Node("alert_subscriber_2"), 
                      high_temp_threshold_(26.0),
                      low_temp_threshold_(20.0)
  {
    // Create subscriber for sensor data (SAME TOPIC as display subscriber)
    sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "sensor_temperature", 10, std::bind(&AlertSubscriber2::sensor_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "🚨 Alert Subscriber 2 started!");
    RCLCPP_INFO(this->get_logger(), "Monitoring: /sensor_temperature");
    RCLCPP_INFO(this->get_logger(), "Alert Thresholds: HIGH > %.1f°C, LOW < %.1f°C", 
                high_temp_threshold_, low_temp_threshold_);
  }

private:
  void sensor_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // Check temperature thresholds and generate alerts
    if (msg->data > high_temp_threshold_) {
      RCLCPP_WARN(this->get_logger(), "🚨 HIGH TEMP ALERT: %.2f°C (Threshold: %.1f°C)", 
                  msg->data, high_temp_threshold_);
    } 
    else if (msg->data < low_temp_threshold_) {
      RCLCPP_WARN(this->get_logger(), "🚨 LOW TEMP ALERT: %.2f°C (Threshold: %.1f°C)", 
                 msg->data, low_temp_threshold_);
    }
    else {
      RCLCPP_INFO(this->get_logger(), "✅ Temperature Normal: %.2f°C", msg->data);
    }
  }
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sensor_sub_;
  const double high_temp_threshold_;
  const double low_temp_threshold_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AlertSubscriber2>());
  rclcpp::shutdown();
  return 0;
}
