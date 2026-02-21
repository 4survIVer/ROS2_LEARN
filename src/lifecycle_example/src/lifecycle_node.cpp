#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

// Define the lifecycle node class
class LifecycleExampleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    LifecycleExampleNode()
    : rclcpp_lifecycle::LifecycleNode("lifecycle_example_node")
    {
        RCLCPP_INFO(this->get_logger(), "Constructor called");
    }

    // Handle transitioning to INACTIVE state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_configure() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Handle transitioning to ACTIVE state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_activate() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Handle transitioning to INACTIVE state
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_deactivate() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Handle node cleanup during shutdown
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_cleanup() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // Handle shutdown
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "on_shutdown() called");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
};

// Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp_lifecycle::LifecycleNode::SharedPtr node = std::make_shared<LifecycleExampleNode>();

    // Create the lifecycle node and spin it to process callbacks
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

