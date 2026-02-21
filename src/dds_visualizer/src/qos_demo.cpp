#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

class QoSDemo : public rclcpp::Node
{
public:
  QoSDemo() : Node("qos_demo")
  {
    // Best Effort publisher
    auto best_effort_qos = rclcpp::QoS(10).best_effort();
    best_effort_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/best_effort_topic", best_effort_qos);
    
    // Reliable publisher  
    auto reliable_qos = rclcpp::QoS(10).reliable();
    reliable_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/reliable_topic", reliable_qos);
    
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        auto message = std_msgs::msg::String();
        message.data = "Hello at " + std::to_string(this->now().seconds());
        
        best_effort_pub_->publish(message);
        reliable_pub_->publish(message);
        
        std::cout << "Published to both topics" << std::endl;
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr best_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reliable_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSDemo>());
  rclcpp::shutdown();
  return 0;
}

