/**
 * DISPLAY SUBSCRIBER 1 - Multiple Subscribers Example
 * 
 * Demonstrates:
 * - Multiple subscribers to same topic
 * - Real-time data display
 * - Subscriber statistics
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <deque>

using std::placeholders::_1;
using namespace std::chrono_literals;


class DisplaySubscriber1 : public rclcpp::Node
{
public:
  DisplaySubscriber1() : Node("display_subscriber_1"), message_count_(0)
  {
    // Create subscriber for sensor data
    sensor_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "sensor_temperature", 10, std::bind(&DisplaySubscriber1::sensor_callback, this, _1));
    
    // Timer for displaying statistics
    stats_timer_ = this->create_wall_timer(2000ms, std::bind(&DisplaySubscriber1::stats_callback, this));
    
    // Initialize statistics
    temp_history_.resize(5, 0.0); // Keep last 5 readings
    
    RCLCPP_INFO(this->get_logger(), "📊 Display Subscriber 1 started!");
    RCLCPP_INFO(this->get_logger(), "Listening to: /sensor_temperature");
  }

private:
  void sensor_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    message_count_++;
    
    // Update temperature history
    temp_history_.pop_front();
    temp_history_.push_back(msg->data);
    
    // Calculate moving average
    double average = 0.0;
    for (auto temp : temp_history_) {
      average += temp;
    }
    average /= temp_history_.size();
    
    RCLCPP_INFO(this->get_logger(), "📈 Current: %.2f°C | Avg (5): %.2f°C", 
                msg->data, average);
  }
  
  void stats_callback()
  {
    RCLCPP_INFO(this->get_logger(), "📊 STATS: Received %d messages in last 2 seconds", 
                message_count_);
    message_count_ = 0; // Reset counter
  }
  
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sensor_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  int message_count_;
  std::deque<double> temp_history_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DisplaySubscriber1>());
  rclcpp::shutdown();
  return 0;
}
