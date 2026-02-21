/**
 * ADD TWO INTS SERVICE SERVER
 * 
 * Demonstrates:
 * - Basic ROS2 service server creation
 * - Request-response pattern
 * - Service callback handling
 * - Synchronous service processing
 */

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer() : Node("add_two_ints_server")
  {
    // Create service for adding two integers
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
      "add_two_ints",
      std::bind(&AddTwoIntsServer::handle_add_two_ints, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "🧮 AddTwoInts Service Server started!");
    RCLCPP_INFO(this->get_logger(), "Service available at: /add_two_ints");
    RCLCPP_INFO(this->get_logger(), "Usage: ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts \"{a: 5, b: 3}\"");
  }

private:
  void handle_add_two_ints(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    // Process the request and compute the sum
    response->sum = request->a + request->b;
    
    RCLCPP_INFO(this->get_logger(), "📥 Received request: %ld + %ld", request->a, request->b);
    RCLCPP_INFO(this->get_logger(), "📤 Sending response: %ld", response->sum);
    
    // Simulate some processing time (optional)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<AddTwoIntsServer>();
  RCLCPP_INFO(node->get_logger(), "AddTwoInts Server ready! Waiting for requests...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
