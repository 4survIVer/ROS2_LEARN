#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>
#include <iostream>
#include <string>
#include <map>
#include <chrono>

namespace dds_visualizer
{

class DDSInfoUtils
{
public:
  static void print_dds_info(rclcpp::Node* node);
  static void print_qos_info(const std::string& topic_name, const rclcpp::QoS& qos);
};

} // namespace dds_visualizer
