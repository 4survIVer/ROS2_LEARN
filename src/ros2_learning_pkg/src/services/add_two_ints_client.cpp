/**
 * ADD TWO INTS SERVICE CLIENT
 * 
 * Demonstrates:
 * - Basic ROS2 service client creation
 * - Synchronous service calls
 * - Request construction and response handling
 * - Error handling for service calls
 */

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class AddTwoIntsClient : public rclcpp::Node
{
public:
  AddTwoIntsClient() : Node("add_two_ints_client")
  {
    // Create client for add_two_ints service
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    
    // Wait for service to be available
    RCLCPP_INFO(this->get_logger(), "🔍 Waiting for service /add_two_ints to be available...");
    
    if (client_->wait_for_service(5s)) {
      RCLCPP_INFO(this->get_logger(), "✅ Service found! Making test requests...");
      make_test_requests();
    } else {
      RCLCPP_ERROR(this->get_logger(), "❌ Service /add_two_ints not available after waiting");
    }
  }

private:
  void make_test_requests()
  {
    // Test case 1: Simple addition
    send_request(5, 3);
    
    // Test case 2: Negative numbers
    send_request(-2, 7);
    
    // Test case 3: Zero values
    send_request(0, 0);
    
    // Test case 4: Large numbers
    send_request(1000, 2500);
  }
  
  void send_request(int64_t a, int64_t b)
  {
    // Create request
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;
    
    RCLCPP_INFO(this->get_logger(), "📤 Sending request: %ld + %ld", a, b);
    
    // Send request synchronously
    auto future = client_->async_send_request(request);
    
    // Wait for the response
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, 3s) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "✅ Result: %ld + %ld = %ld", a, b, response->sum);
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "❌ Failed to call service /add_two_ints");
    }
    
    // Small delay between requests
    std::this_thread::sleep_for(500ms);
  }
  
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<AddTwoIntsClient>();
  
  // Keep the node alive for a bit to see all responses
  std::this_thread::sleep_for(5s);
  
  rclcpp::shutdown();
  return 0;
}
