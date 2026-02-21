#include "rclcpp/rclcpp.hpp"
#include <iostream>

class MinimalDDSMonitor : public rclcpp::Node
{
public:
  MinimalDDSMonitor() : Node("minimal_dds_monitor")
  {
    // Simple node that just prints
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      [this]() {
        auto nodes = this->get_node_names();
        std::cout << "Discovered " << nodes.size() << " nodes" << std::endl;
      });
    std::cout << "Minimal DDS Monitor Started!" << std::endl;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalDDSMonitor>());
  rclcpp::shutdown();
  return 0;
}

