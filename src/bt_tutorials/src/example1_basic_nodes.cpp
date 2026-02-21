/**
 * @file example1_basic_nodes.cpp
 * @brief Example 1: Basic Behavior Tree Nodes
 * 
 * This example demonstrates:
 * 1. Creating custom Action and Condition nodes
 * 2. Simple tree structure
 * 3. Basic tree execution
 * 
 * Behavior Tree Structure:
 *   - Sequence
 *     - Condition: IsBatteryOK
 *     - Action:   MoveToGoal
 *     - Action:   PerformTask
 */

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

// Forward declarations for our custom nodes
namespace BT_Tutorials
{

// ============================================================================
// CUSTOM ACTION NODE: MoveToGoal
// ============================================================================
/**
 * @class MoveToGoal
 * @brief Action node that simulates moving to a goal
 * 
 * Action nodes typically represent activities that take time to complete.
 * They return RUNNING while executing, SUCCESS when done, or FAILURE if something goes wrong.
 * 
 * In BehaviorTree.CPP, nodes are created by inheriting from SyncActionNode, 
 * AsyncActionNode, or ConditionNode.
 */
class MoveToGoal : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for MoveToGoal node
   * @param name Name of the node
   * @param config Node configuration (ports, etc.)
   * 
   * All BT nodes receive these parameters. The name appears in the XML tree.
   */
  MoveToGoal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    // Initialize any resources here
    RCLCPP_INFO(rclcpp::get_logger("MoveToGoal"), "Node created: %s", name.c_str());
  }

  /**
   * @brief Define the ports (inputs/outputs) for this node
   * 
   * Ports allow data flow between nodes via the Blackboard.
   * InputPort: Read data from Blackboard
   * OutputPort: Write data to Blackboard
   */
  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("goal_location"),  // Input: Where to move
      BT::OutputPort<double>("distance_traveled")   // Output: How far we moved
    };
  }

  /**
   * @brief The main execution function
   * @return NodeStatus (SUCCESS, FAILURE, or RUNNING)
   * 
   * For SyncActionNode, this runs once and returns immediately.
   * For long-running tasks, use AsyncActionNode instead.
   */
  BT::NodeStatus tick() override
  {
    // 1. Get input from port (if any)
    std::string goal;
    if (!getInput<std::string>("goal_location", goal))
    {
      // If required input is missing, return FAILURE
      RCLCPP_ERROR(rclcpp::get_logger("MoveToGoal"), "goal_location port is missing!");
      return BT::NodeStatus::FAILURE;
    }

    // 2. Perform the action
    RCLCPP_INFO(rclcpp::get_logger("MoveToGoal"), 
                "Moving to goal: %s", goal.c_str());
    
    // Simulate some work
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // 3. Set output on port (if any)
    double distance = 10.5;  // Simulated distance
    setOutput("distance_traveled", distance);
    
    // 4. Return status
    RCLCPP_INFO(rclcpp::get_logger("MoveToGoal"), 
                "Arrived at %s. Distance: %.1f meters", goal.c_str(), distance);
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// CUSTOM ACTION NODE: PerformTask
// ============================================================================
class PerformTask : public BT::SyncActionNode
{
public:
  PerformTask(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("task_type")  // What task to perform
    };
  }

  BT::NodeStatus tick() override
  {
    std::string task;
    if (!getInput<std::string>("task_type", task))
    {
      task = "unknown";
    }

    RCLCPP_INFO(rclcpp::get_logger("PerformTask"), 
                "Performing task: %s", task.c_str());
    
    // Simulate task execution
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    
    // Sometimes tasks fail (simulate 20% failure rate)
    if (rand() % 100 < 20)
    {
      RCLCPP_WARN(rclcpp::get_logger("PerformTask"), 
                  "Task failed: %s", task.c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PerformTask"), 
                "Task completed: %s", task.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// CUSTOM CONDITION NODE: IsBatteryOK
// ============================================================================
/**
 * @class IsBatteryOK
 * @brief Condition node that checks a condition
 * 
 * Condition nodes are quick checks that return SUCCESS or FAILURE.
 * They should not block or take significant time.
 */
class IsBatteryOK : public BT::ConditionNode
{
public:
  IsBatteryOK(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), battery_level_(75.0)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("min_battery")  // Minimum required battery
    };
  }

  BT::NodeStatus tick() override
  {
    // Get minimum battery level from port (with default value)
    double min_battery = 30.0;
    getInput<double>("min_battery", min_battery);

    // Check condition
    if (battery_level_ >= min_battery)
    {
      RCLCPP_INFO(rclcpp::get_logger("IsBatteryOK"), 
                  "Battery OK: %.1f%% (minimum: %.1f%%)", 
                  battery_level_, min_battery);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("IsBatteryOK"), 
                  "Battery LOW: %.1f%% (minimum: %.1f%%)", 
                  battery_level_, min_battery);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  double battery_level_;  // Simulated battery level
};

}  // namespace BT_Tutorials

// ============================================================================
// MAIN FUNCTION
// ============================================================================
int main(int argc, char **argv)
{
  // 1. Initialize ROS2
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_example1");
  
  RCLCPP_INFO(node->get_logger(), "Starting Behavior Tree Example 1: Basic Nodes");

  // 2. Create Behavior Tree Factory
  /**
   * The factory is used to register node types and create trees from XML.
   * It's the central registry for all your custom nodes.
   */
  BT::BehaviorTreeFactory factory;

  // 3. Register our custom nodes with the factory
  /**
   * Each node type must be registered before it can be used in XML.
   * The template parameter is the node class name.
   */
  factory.registerNodeType<BT_Tutorials::MoveToGoal>("MoveToGoal");
  factory.registerNodeType<BT_Tutorials::PerformTask>("PerformTask");
  factory.registerNodeType<BT_Tutorials::IsBatteryOK>("IsBatteryOK");

  // 4. Define the Behavior Tree in XML
  /**
   * XML structure:
   * - <root> is the root tag
   * - <BehaviorTree> defines the tree
   * - Control nodes (Sequence, Fallback, Parallel) organize flow
   * - Action/Condition nodes do the work
   * 
   * This tree: Sequence (→ means "then")
   *   IsBatteryOK → MoveToGoal → PerformTask
   * 
   * Sequence: Executes children in order. Fails if ANY child fails.
   */
  const std::string xml_tree = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
          <!-- Check battery first -->
          <IsBatteryOK min_battery="20.0"/>
          
          <!-- Move to location -->
          <MoveToGoal goal_location="kitchen" 
                     distance_traveled="{distance}"/>
          
          <!-- Perform the task -->
          <PerformTask task_type="deliver_item"/>
        </Sequence>
      </BehaviorTree>
    </root>
  )";

  // 5. Create the tree from XML
  /**
   * The factory parses XML and creates node instances.
   * The blackboard is shared memory for data exchange between nodes.
   */
  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromText(xml_tree, blackboard);
  
  // 6. Add a logger to see what's happening
  /**
   * BT::StdCoutLogger prints node status changes to console.
   * Useful for debugging and understanding tree execution.
   */
  BT::StdCoutLogger logger(tree);

  // 7. Execute the tree
  RCLCPP_INFO(node->get_logger(), "Starting tree execution...");
  RCLCPP_INFO(node->get_logger(), "==================================");
  
  /**
   * Tree execution:
   * - tick() executes the tree once (from root)
   * - Returns when tree reaches a terminal state (SUCCESS/FAILURE)
   * - For continuous execution, tick in a loop
   */
  BT::NodeStatus status = tree.tickRoot();
  
  RCLCPP_INFO(node->get_logger(), "==================================");
  RCLCPP_INFO(node->get_logger(), "Tree finished with status: %s",
              (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");

  // 8. Check blackboard data
  /**
   * The blackboard stores data shared between nodes.
   * We can retrieve values set by nodes during execution.
   */
  auto distance = blackboard->get<double>("distance");
  RCLCPP_INFO(node->get_logger(), "Distance traveled: %.1f meters", distance);

  // 9. Cleanup
  rclcpp::shutdown();
  return 0;
}
