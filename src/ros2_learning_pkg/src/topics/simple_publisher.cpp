/**
 * SIMPLE PUBLISHER EXAMPLE
 * 
 * This node demonstrates:
 * - Basic ROS2 node creation
 * - Publisher creation and message publishing
 * - Timer-based callbacks
 * - String message usage
 * 
 * Key Concepts:
 * - Nodes: Fundamental ROS2 executable
 * - Topics: Named buses for message exchange  
 * - Publishers: Send messages to topics
 * - Messages: Data structures for communication
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher() : Node("simple_publisher")
  {
    // Create a publisher on topic "chatter" with queue size 10
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    
    // Create a timer that triggers every 500 milliseconds
    timer_ = this->create_wall_timer(
      500ms,  // Timer period
      std::bind(&SimplePublisher::timer_callback, this)  // Callback function
    );
    
    RCLCPP_INFO(this->get_logger(), "Simple Publisher started! Publishing on topic: /chatter");
  }

private:
  void timer_callback()
  {
    // Create a String message
    auto message = std_msgs::msg::String();
    message.data = "Hello ROS2 World! Time: " + std::to_string(this->now().seconds());
    
    // Publish the message
    publisher_->publish(message);
    
    // Log the published message
    RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
  }
  
  // Member variables
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create and spin the node
  auto node = std::make_shared<SimplePublisher>();
  rclcpp::spin(node);
  
  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}

