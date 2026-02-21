#include "dds_visualizer/dds_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>

namespace dds_visualizer
{

// Implementation of DDSInfoUtils methods
void DDSInfoUtils::print_dds_info(rclcpp::Node* node)
{
    std::cout << "=== DDS MIDDLEWARE INFORMATION ===" << std::endl;
    std::cout << "RMW Implementation: " << rmw_get_implementation_identifier() << std::endl;
    std::cout << "Node Name: " << node->get_name() << std::endl;
    std::cout << "Namespace: " << node->get_namespace() << std::endl;
    
    // Get topic information
    auto topic_names_and_types = node->get_topic_names_and_types();
    std::cout << "Number of active topics: " << topic_names_and_types.size() << std::endl;
    std::cout << "=================================" << std::endl;
}

void DDSInfoUtils::print_qos_info(const std::string& topic_name, const rclcpp::QoS& qos)
{
    std::cout << "=== QoS Settings for '" << topic_name << "' ===" << std::endl;
    std::cout << "Reliability: " << 
        (qos.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "RELIABLE" : "BEST_EFFORT") << std::endl;
    std::cout << "Durability: " << 
        (qos.durability() == rclcpp::DurabilityPolicy::TransientLocal ? "TRANSIENT_LOCAL" : "VOLATILE") << std::endl;
    std::cout << "History Depth: " << qos.get_rmw_qos_profile().depth << std::endl;
    std::cout << "=================================" << std::endl;
}

} // namespace dds_visualizer
