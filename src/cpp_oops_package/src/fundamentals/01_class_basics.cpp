/**
 * C++ CLASS FUNDAMENTALS WITH ROS2
 * 
 * This example demonstrates:
 * - Basic class structure in C++
 * - Constructors and member initializer lists  
 * - Member variables vs local variables
 * - Access specifiers (public, private)
 * - Smart pointers and memory management
 * - ROS2 node as a class
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <memory>  // For std::make_shared
#include <string>  // For std::to_string, std::string

using namespace std::chrono_literals;

/**
 * @class DetailedPublisher
 * @brief Demonstrates C++ class concepts with ROS2 publisher
 * 
 * KEY C++ CONCEPTS:
 * - Class Declaration: 'class DetailedPublisher'
 * - Inheritance: ': public rclcpp::Node' 
 * - Access Specifiers: 'public:', 'private:'
 * - Constructor: 'DetailedPublisher()'
 * - Member Initializer List: ': Node("detailed_publisher")'
 * - Member Variables: 'publisher_', 'timer_', 'counter_'
 * - Member Functions: 'timer_callback()'
 * - Smart Pointers: 'SharedPtr' types
 */
class DetailedPublisher : public rclcpp::Node
{
public:
  /**
   * @brief Constructor - called when object is created
   * 
   * C++ CONSTRUCTOR FEATURES:
   * - Same name as class
   * - No return type
   * - Member initializer list (after colon) for efficient initialization
   * - Body in braces for complex initialization
   */
  DetailedPublisher() 
  : Node("detailed_publisher"),  // Call parent class constructor
    counter_(0)                  // Initialize member variable
  {
    // STEP 1: Create publisher
    // 'this->' refers to current object (like self in Python)
    // 'create_publisher' returns SharedPtr (smart pointer)
    // Template parameter <std_msgs::msg::String> specifies message type
    // "chatter" is topic name, 10 is queue size
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    
    // STEP 2: Create timer
    // 1000ms = 1 second period
    // std::bind connects member function to timer
    timer_ = this->create_wall_timer(
      1000ms,  // Period: 1000 milliseconds = 1 second
      std::bind(&DetailedPublisher::timer_callback, this)  // Callback binding
    );
    
    // STEP 3: Log initialization
    // RCLCPP_INFO is ROS2 logging macro
    // this->get_logger() gets node's logger
    RCLCPP_INFO(this->get_logger(), 
                "DetailedPublisher initialized! Publishing to: /chatter");
  }

private:
  /**
   * @brief Timer callback function - called every timer period
   * 
   * C++ FUNCTION CONCEPTS:
   * - 'void' return type: doesn't return anything
   * - Access: 'private' - can only be called from within class
   * - Member function: has access to all class members
   */
  void timer_callback()
  {
    // STEP 1: Create message object
    // 'auto' keyword: compiler deduces type automatically
    // Equivalent to: std_msgs::msg::String message;
    auto message = std_msgs::msg::String();
    
    // STEP 2: Set message data
    // message.data is a std::string
    // std::to_string() converts numbers to strings
    // this->now() gets current ROS time
    // this->counter_ accesses member variable
    message.data = "Message #" + std::to_string(counter_++) + 
                   " at time: " + std::to_string(this->now().seconds());
    
    // STEP 3: Publish the message
    // publisher_ is SharedPtr, so use -> to access methods
    publisher_->publish(message);
    
    // STEP 4: Log the publication
    // message.data.c_str() converts std::string to C-style string for printf
    RCLCPP_INFO(this->get_logger(), "Published: '%s'", message.data.c_str());
  }
  
  // MEMBER VARIABLE DECLARATIONS
  // These exist for the lifetime of the object
  
  /**
   * @brief Publisher as shared pointer
   * 
   * SMART POINTER EXPLANATION:
   * - SharedPtr automatically manages memory
   * - No need for manual new/delete
   * - Multiple objects can share ownership
   * - Automatically deleted when last reference is gone
   */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  
  /**
   * @brief Timer as shared pointer  
   */
  rclcpp::TimerBase::SharedPtr timer_;
  
  /**
   * @brief Counter variable to track messages
   * 
   * MEMBER VARIABLE NOTES:
   * - Underscore suffix is naming convention
   * - Initialized in constructor's member initializer list
   * - Exists for object lifetime (not just function scope)
   */
  int counter_;
};

/**
 * @brief Main function - program entry point
 * 
 * C++ MAIN FUNCTION:
 * - Standard C/C++ entry point
 * - argc: argument count
 * - argv: argument values
 * - Returns int (0 = success)
 */
int main(int argc, char * argv[])
{
  // STEP 1: Initialize ROS2 context
  // Must be called before any ROS2 operations
  rclcpp::init(argc, argv);
  
  // STEP 2: Create node object using smart pointer
  // std::make_shared: modern C++ way to create objects
  // Automatically manages memory - no manual deletion needed
  auto node = std::make_shared<DetailedPublisher>();
  
  // STEP 3: Keep node running and processing callbacks
  // rclcpp::spin blocks until node is shutdown
  // Processes timers, subscriptions, services, etc.
  rclcpp::spin(node);
  
  // STEP 4: Cleanup ROS2 resources
  // Called after spin returns (when node is shutdown)
  rclcpp::shutdown();
  
  // STEP 5: Exit program
  return 0;
}
