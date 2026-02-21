/**
 * SIMPLE SUBSCRIBER EXAMPLE
 * 
 * This node demonstrates:
 * - Basic ROS2 node creation  
 * - Subscriber creation and message reception
 * - Callback functions for incoming messages
 * - String message processing
 * 
 * Key Concepts:
 * - Subscribers: Receive messages from topics
 * - Callbacks: Functions triggered by incoming messages
 * - Message processing: Handling received data
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node
{
public:
  SimpleSubscriber() : Node("simple_subscriber")
  {
    // Create a subscriber on topic "chatter" with queue size 10
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "chatter",  // Topic name
      10,         // Queue size
      std::bind(&SimpleSubscriber::topic_callback, this, _1)  // Callback function
    );
    
    RCLCPP_INFO(this->get_logger(), "Simple Subscriber started! Listening on topic: /chatter");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Process the incoming message
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  
  // Member variable
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  
  // Create and spin the node
  auto node = std::make_shared<SimpleSubscriber>();
  rclcpp::spin(node);
  
  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}

