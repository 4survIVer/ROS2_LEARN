/**
 * CUSTOM MESSAGE SUBSCRIBER EXAMPLE
 * 
 * Demonstrates:
 * - Subscribing to custom message types
 * - Processing complex data structures
 * - Multiple topic subscription in one node
 */

#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_pkg/msg/student_info.hpp"
#include "ros2_learning_pkg/msg/vector_stamped.hpp"

using std::placeholders::_1;

class CustomMessageSubscriber : public rclcpp::Node
{
public:
  CustomMessageSubscriber() : Node("custom_message_subscriber")
  {
    // Create subscribers for custom messages
    student_sub_ = this->create_subscription<ros2_learning_pkg::msg::StudentInfo>(
      "student_info", 10, std::bind(&CustomMessageSubscriber::student_callback, this, _1));
    
    vector_sub_ = this->create_subscription<ros2_learning_pkg::msg::VectorStamped>(
      "vector_data", 10, std::bind(&CustomMessageSubscriber::vector_callback, this, _1));
    
    RCLCPP_INFO(this->get_logger(), "Custom Message Subscriber started!");
    RCLCPP_INFO(this->get_logger(), "Listening on topics: /student_info and /vector_data");
  }

private:
  void student_callback(const ros2_learning_pkg::msg::StudentInfo::SharedPtr msg)
  {
    std::string student_type = msg->is_undergraduate ? "Undergraduate" : "Graduate";
    
    RCLCPP_INFO(this->get_logger(), "📚 Received Student: %s %s", 
                msg->first_name.c_str(), msg->last_name.c_str());
    RCLCPP_INFO(this->get_logger(), "   Age: %d, GPA: %.2f, Major: %s, Type: %s",
                msg->age, msg->gpa, msg->major.c_str(), student_type.c_str());
  }
  
  void vector_callback(const ros2_learning_pkg::msg::VectorStamped::SharedPtr msg)
  {
    auto sec = msg->stamp.sec;
    auto nanosec = msg->stamp.nanosec;
    
    RCLCPP_INFO(this->get_logger(), "📐 Received Vector: (%.2f, %.2f, %.2f)", 
                msg->x, msg->y, msg->z);
    RCLCPP_INFO(this->get_logger(), "   Frame: %s, Time: %d.%09d",
                msg->frame_id.c_str(), sec, nanosec);
  }
  
  rclcpp::Subscription<ros2_learning_pkg::msg::StudentInfo>::SharedPtr student_sub_;
  rclcpp::Subscription<ros2_learning_pkg::msg::VectorStamped>::SharedPtr vector_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomMessageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
