/**
 * @file example5_subtrees.cpp
 * @brief Example 5: SubTrees and XML File Loading
 * FINAL WORKING VERSION
 */

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <fstream>

namespace BT_Tutorials
{

// ============================================================================
// BASIC ACTION NODES
// ============================================================================

class AnnounceAction : public BT::SyncActionNode
{
public:
  AnnounceAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    std::string msg;
    if (!getInput("message", msg).has_value())
    {
      RCLCPP_WARN(rclcpp::get_logger("AnnounceAction"), "No message provided");
      msg = "Default message";
    }
    
    RCLCPP_INFO(rclcpp::get_logger("AnnounceAction"), 
                "📢 %s", msg.c_str());
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    
    return BT::NodeStatus::SUCCESS;
  }
};

class MoveToAction : public BT::SyncActionNode
{
public:
  MoveToAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("target"),
      BT::OutputPort<double>("distance_output")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string target;
    if (!getInput("target", target).has_value())
    {
      RCLCPP_ERROR(rclcpp::get_logger("MoveToAction"), "No target specified!");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("MoveToAction"), 
                "🚶 Moving to: %s", target.c_str());
    
    // Simulate movement time
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // Simulate distance traveled
    double distance = 10.0 + (rand() % 20);
    setOutput("distance_output", distance);
    
    RCLCPP_INFO(rclcpp::get_logger("MoveToAction"), 
                "✅ Arrived at %s (Distance: %.1f meters)", 
                target.c_str(), distance);
    
    return BT::NodeStatus::SUCCESS;
  }
};

class WaitAction : public BT::SyncActionNode
{
public:
  WaitAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("duration") };
  }

  BT::NodeStatus tick() override
  {
    int duration = 1;
    getInput("duration", duration);
    
    RCLCPP_INFO(rclcpp::get_logger("WaitAction"), 
                "⏳ Waiting for %d seconds...", duration);
    
    rclcpp::sleep_for(std::chrono::seconds(duration));
    
    RCLCPP_INFO(rclcpp::get_logger("WaitAction"), "✅ Wait complete");
    
    return BT::NodeStatus::SUCCESS;
  }
};

class PerformTaskAction : public BT::SyncActionNode
{
public:
  PerformTaskAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("task_type"),
      BT::OutputPort<bool>("result")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string task;
    if (!getInput("task_type", task).has_value())
    {
      RCLCPP_ERROR(rclcpp::get_logger("PerformTaskAction"), 
                   "No task specified!");
      setOutput("result", false);
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PerformTaskAction"), 
                "🔧 Performing task: %s", task.c_str());
    
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    
    // Simulate task success (70% success rate)
    bool success = (rand() % 100 < 70);
    setOutput("result", success);
    
    if (success)
    {
      RCLCPP_INFO(rclcpp::get_logger("PerformTaskAction"), 
                  "✅ Task '%s' completed successfully", task.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("PerformTaskAction"), 
                  "❌ Task '%s' failed!", task.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

class CheckBattery : public BT::ConditionNode
{
public:
  CheckBattery(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), battery_level_(75.0)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("min_level"),
      BT::OutputPort<bool>("battery_status")
    };
  }

  BT::NodeStatus tick() override
  {
    double min_level = 20.0;
    getInput("min_level", min_level);
    
    // Simulate battery drain
    battery_level_ -= 0.5;
    if (battery_level_ < 0) battery_level_ = 100.0;
    
    bool is_ok = (battery_level_ >= min_level);
    setOutput("battery_status", is_ok);
    
    if (is_ok)
    {
      RCLCPP_INFO(rclcpp::get_logger("CheckBattery"), 
                  "🔋 Battery OK: %.1f%%", battery_level_);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("CheckBattery"), 
                  "🔋 Battery LOW: %.1f%%", battery_level_);
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  double battery_level_;
};

}  // namespace BT_Tutorials

// Helper function
std::string loadXMLFromFile(const std::string& filepath)
{
  std::ifstream file(filepath);
  if (!file.is_open())
  {
    throw std::runtime_error("Cannot open file: " + filepath);
  }
  std::string content((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());
  return content;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_example5");
  
  RCLCPP_INFO(node->get_logger(), 
              "Starting Behavior Tree Example 5: SubTrees and XML Files");
  RCLCPP_INFO(node->get_logger(), 
              "==============================================================");
  
  srand(time(nullptr));

  BT::BehaviorTreeFactory factory;

  // Register all nodes
  factory.registerNodeType<BT_Tutorials::AnnounceAction>("AnnounceAction");
  factory.registerNodeType<BT_Tutorials::MoveToAction>("MoveToAction");
  factory.registerNodeType<BT_Tutorials::WaitAction>("WaitAction");
  factory.registerNodeType<BT_Tutorials::PerformTaskAction>("PerformTaskAction");
  factory.registerNodeType<BT_Tutorials::CheckBattery>("CheckBattery");

  // ============================================
  // PART 1: Loading Subtrees from File
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 1: Loading Subtrees from File ===");
    
    try
    {
      std::string subtrees_xml = loadXMLFromFile("trees/subtree_components.xml");
      factory.registerBehaviorTreeFromText(subtrees_xml);
      RCLCPP_INFO(node->get_logger(), "✅ Successfully loaded subtree components");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(node->get_logger(), "Could not load XML: %s", e.what());
    }
  }

  // ============================================
  // PART 2: Simple Tree
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 2: Simple Tree ===");
    
    const std::string simple_tree = R"(
      <root>
        <BehaviorTree ID="SimpleTree">
          <Sequence>
            <AnnounceAction message="Starting simple demo"/>
            <MoveToAction target="kitchen"/>
            <WaitAction duration="1"/>
            <MoveToAction target="bedroom"/>
            <AnnounceAction message="Simple demo complete"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerBehaviorTreeFromText(simple_tree);
    auto tree = factory.createTree("SimpleTree");
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing simple tree...");
    auto status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Simple tree result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 3: Working Example (No port issues)
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 3: Working Example ===");
    
    const std::string working_tree = R"(
      <root>
        <BehaviorTree ID="WorkingExample">
          <Sequence>
            <AnnounceAction message="=== Working Example Start ==="/>
            
            <Sequence>
              <AnnounceAction message="Going to kitchen"/>
              <MoveToAction target="kitchen"/>
              <WaitAction duration="1"/>
            </Sequence>
            
            <Sequence>
              <AnnounceAction message="Performing kitchen task"/>
              <PerformTaskAction task_type="clean"/>
            </Sequence>
            
            <Sequence>
              <AnnounceAction message="Returning to base"/>
              <MoveToAction target="base"/>
            </Sequence>
            
            <AnnounceAction message="=== Working Example Complete ==="/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerBehaviorTreeFromText(working_tree);
    auto tree = factory.createTree("WorkingExample");
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing working example...");
    auto status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Working example result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 4: Simple Nested Subtrees (Working)
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 4: Simple Nested Subtrees ===");
    
    const std::string simple_nested = R"(
      <root>
        <BehaviorTree ID="LeafAction">
          <Sequence>
            <AnnounceAction message="Leaf: Performing basic action"/>
            <PerformTaskAction task_type="default_task"/>
          </Sequence>
        </BehaviorTree>
        
        <BehaviorTree ID="MiddleLevel">
          <Sequence>
            <AnnounceAction message="Middle: Starting"/>
            <SubTree ID="LeafAction"/>
            <WaitAction duration="1"/>
            <SubTree ID="LeafAction"/>
            <AnnounceAction message="Middle: Complete"/>
          </Sequence>
        </BehaviorTree>
        
        <BehaviorTree ID="TopLevel">
          <Sequence>
            <AnnounceAction message="=== Top Level Start ==="/>
            <SubTree ID="MiddleLevel"/>
            <AnnounceAction message="=== Top Level Complete ==="/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerBehaviorTreeFromText(simple_nested);
    
    const std::string use_nested = R"(
      <root>
        <BehaviorTree ID="UseNested">
          <Sequence>
            <AnnounceAction message="=== Using Nested Structure ==="/>
            <SubTree ID="TopLevel"/>
            <AnnounceAction message="=== All Done ==="/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerBehaviorTreeFromText(use_nested);
    auto tree = factory.createTree("UseNested");
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing nested structure...");
    auto status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Nested structure result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 5: Tree Validation
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 5: Tree Validation ===");
    
    const std::string inspect_tree = R"(
      <root>
        <BehaviorTree ID="InspectTree">
          <Sequence>
            <AnnounceAction message="Tree inspection starting"/>
            <MoveToAction target="test_location"/>
            <CheckBattery min_level="30"/>
            <AnnounceAction message="Inspection complete"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerBehaviorTreeFromText(inspect_tree);
    auto tree = factory.createTree("InspectTree");
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing inspection tree...");
    tree.tickRoot();
  }

  RCLCPP_INFO(node->get_logger(), 
              "\n==============================================================");
  RCLCPP_INFO(node->get_logger(), "Example 5 completed successfully!");
  
  rclcpp::shutdown();
  return 0;
}