/**
 * CUSTOM MESSAGE PUBLISHER EXAMPLE
 * 
 * Demonstrates:
 * - Creating and using custom message types
 * - Complex data structures in ROS2 messages
 * - Custom message publishing with timers
 */

#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_pkg/msg/student_info.hpp"
#include "ros2_learning_pkg/msg/vector_stamped.hpp"
#include <chrono>

using namespace std::chrono_literals;

class CustomMessagePublisher : public rclcpp::Node
{
public:
  CustomMessagePublisher() : Node("custom_message_publisher"), student_id_(1000)
  {
    // Create publishers for custom messages
    student_pub_ = this->create_publisher<ros2_learning_pkg::msg::StudentInfo>("student_info", 10);
    vector_pub_ = this->create_publisher<ros2_learning_pkg::msg::VectorStamped>("vector_data", 10);
    
    // Timer for publishing custom messages
    timer_ = this->create_wall_timer(1000ms, std::bind(&CustomMessagePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Custom Message Publisher started!");
    RCLCPP_INFO(this->get_logger(), "Publishing on topics: /student_info and /vector_data");
  }

private:
  void timer_callback()
  {
    publish_student_data();
    publish_vector_data();
  }
  
  void publish_student_data()
  {
    auto student_msg = ros2_learning_pkg::msg::StudentInfo();
    
    // Fill student data
    student_msg.first_name = "John";
    student_msg.last_name = "Doe";
    student_msg.age = 20 + (student_id_ % 5);
    student_msg.gpa = 3.5 + (student_id_ % 10) * 0.1;
    student_msg.major = "Computer Science";
    student_msg.is_undergraduate = true;
    
    student_pub_->publish(student_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published Student: %s %s, Age: %d, GPA: %.2f", 
                student_msg.first_name.c_str(), student_msg.last_name.c_str(),
                student_msg.age, student_msg.gpa);
    
    student_id_++;
  }
  
  void publish_vector_data()
  {
    auto vector_msg = ros2_learning_pkg::msg::VectorStamped();
    
    // Fill vector data with current time
    vector_msg.stamp = this->now();
    vector_msg.x = 1.0 + (student_id_ % 10) * 0.1;
    vector_msg.y = 2.0 + (student_id_ % 10) * 0.1;
    vector_msg.z = 3.0 + (student_id_ % 10) * 0.1;
    vector_msg.frame_id = "map";
    
    vector_pub_->publish(vector_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published Vector: (%.2f, %.2f, %.2f) in frame '%s'",
                vector_msg.x, vector_msg.y, vector_msg.z, vector_msg.frame_id.c_str());
  }
  
  rclcpp::Publisher<ros2_learning_pkg::msg::StudentInfo>::SharedPtr student_pub_;
  rclcpp::Publisher<ros2_learning_pkg::msg::VectorStamped>::SharedPtr vector_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t student_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomMessagePublisher>());
  rclcpp::shutdown();
  return 0;
}
