#include "dds_visualizer/dds_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <map>
#include <chrono>

class DDSDiscoveryMonitor : public rclcpp::Node
{
public:
  DDSDiscoveryMonitor() : Node("dds_discovery_monitor")
  {
    // Print DDS information
    dds_visualizer::DDSInfoUtils::print_dds_info(this);
    
    // Create publisher for visualization
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/dds_discovery", rclcpp::QoS(10).reliable().transient_local());
    
    // Monitor timer
    monitor_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&DDSDiscoveryMonitor::discover_nodes_and_topics, this));
    
    RCLCPP_INFO(this->get_logger(), "DDS Discovery Monitor Started!");
  }

private:
  void discover_nodes_and_topics()
  {
    auto node_names = this->get_node_names();
    auto topic_names_and_types = this->get_topic_names_and_types();
    
    std::cout << "\n=== DDS DISCOVERY UPDATE ===" << std::endl;
    std::cout << "Discovered Nodes: " << node_names.size() << std::endl;
    for (const auto& node : node_names) {
      std::cout << "  - " << node << std::endl;
    }
    
    std::cout << "Discovered Topics: " << topic_names_and_types.size() << std::endl;
    for (const auto& [topic_name, types] : topic_names_and_types) {
      std::cout << "  - " << topic_name << " [";
      for (const auto& type : types) {
        std::cout << type << " ";
      }
      std::cout << "]" << std::endl;
    }
  }
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DDSDiscoveryMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

