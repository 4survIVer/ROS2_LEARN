/**
 * @file example2_control_flow.cpp
 * @brief Example 2: Control Flow Nodes - ISOLATED VERSION
 * 
 * Each example runs in complete isolation to avoid logger conflicts
 */

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

namespace BT_Tutorials
{

// Simple action nodes for demonstration
class ActionA : public BT::SyncActionNode
{
public:
  ActionA(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), call_count_(0)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    call_count_++;
    RCLCPP_INFO(rclcpp::get_logger("ActionA"), 
                "ActionA executed (call #%d)", call_count_);
    
    if (call_count_ == 1)
    {
      RCLCPP_WARN(rclcpp::get_logger("ActionA"), "ActionA failing on first try");
      return BT::NodeStatus::FAILURE;
    }
    
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    return BT::NodeStatus::SUCCESS;
  }

private:
  int call_count_;
};

class ActionB : public BT::SyncActionNode
{
public:
  ActionB(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("ActionB"), "ActionB executed");
    rclcpp::sleep_for(std::chrono::milliseconds(150));
    return BT::NodeStatus::SUCCESS;
  }
};

class ActionC : public BT::SyncActionNode
{
public:
  ActionC(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("ActionC"), "ActionC executed");
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    return BT::NodeStatus::FAILURE;  // Always fails for demo
  }
};

}  // namespace BT_Tutorials

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_example2");
  
  RCLCPP_INFO(node->get_logger(), 
              "Starting Behavior Tree Example 2: Control Flow");

  // ============================================
  // PART 1: SEQUENCE vs FALLBACK
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 1: Sequence vs Fallback ===");
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT_Tutorials::ActionA>("ActionA");
    factory.registerNodeType<BT_Tutorials::ActionB>("ActionB");
    factory.registerNodeType<BT_Tutorials::ActionC>("ActionC");
    
    // Test Sequence
    RCLCPP_INFO(node->get_logger(), "\n[SEQUENCE]: All must succeed");
    const std::string xml_sequence = R"(
      <root main_tree_to_execute="SequenceExample">
        <BehaviorTree ID="SequenceExample">
          <Sequence name="seq">
            <ActionA/>
            <ActionB/>
            <ActionC/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    auto tree_seq = factory.createTreeFromText(xml_sequence);
    // Use RCLCPP logging instead of StdCoutLogger
    auto status_seq = tree_seq.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Sequence result: %s", 
                (status_seq == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
    
    // Test Fallback  
    RCLCPP_INFO(node->get_logger(), "\n[FALLBACK]: First success wins");
    const std::string xml_fallback = R"(
      <root main_tree_to_execute="FallbackExample">
        <BehaviorTree ID="FallbackExample">
          <Fallback name="fallback">
            <ActionA/>
            <ActionB/>
            <ActionC/>
          </Fallback>
        </BehaviorTree>
      </root>
    )";
    
    auto tree_fal = factory.createTreeFromText(xml_fallback);
    auto status_fal = tree_fal.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Fallback result: %s", 
                (status_fal == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }
  
  // ============================================
  // PART 2: PARALLEL (separate factory to avoid conflicts)
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 2: Parallel Node ===");
    
    // Create fresh factory for parallel example
    BT::BehaviorTreeFactory factory2;
    
    // Re-register nodes
    factory2.registerNodeType<BT_Tutorials::ActionA>("ActionA");
    
    // Special node for parallel example
    class LongAction : public BT::AsyncActionNode
    {
    public:
      LongAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config), completed_(false)
      {}

      static BT::PortsList providedPorts()
      {
        return { BT::InputPort<int>("duration_ms") };
      }

      BT::NodeStatus tick() override
      {
        int duration = 1000;
        getInput<int>("duration_ms", duration);
        
        RCLCPP_INFO(rclcpp::get_logger("LongAction"), 
                    "Starting long action (%d ms)", duration);
        
        future_ = std::async(std::launch::async, [this, duration]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(duration));
          completed_ = true;
        });
        
        return BT::NodeStatus::RUNNING;
      }

      void halt() override
      {
        RCLCPP_WARN(rclcpp::get_logger("LongAction"), "LongAction halted!");
        completed_ = true;
      }

      bool isCompleted() const { return completed_; }

    private:
      std::future<void> future_;
      bool completed_;
    };
    
    factory2.registerNodeType<LongAction>("LongAction");
    
    const std::string xml_parallel = R"(
      <root main_tree_to_execute="ParallelExample">
        <BehaviorTree ID="ParallelExample">
          <Parallel success_threshold="2" failure_threshold="2">
            <LongAction duration_ms="800"/>
            <LongAction duration_ms="1200"/>
            <ActionA/>
          </Parallel>
        </BehaviorTree>
      </root>
    )";

    auto tree_par = factory2.createTreeFromText(xml_parallel);
    // Use ONE StdCoutLogger for this section only
    BT::StdCoutLogger logger_par(tree_par);
    
    RCLCPP_INFO(node->get_logger(), "Starting parallel execution...");
    auto status = BT::NodeStatus::RUNNING;
    int tick_count = 0;
    
    while (status == BT::NodeStatus::RUNNING && tick_count < 10)
    {
      status = tree_par.tickRoot();
      tick_count++;
      rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
    
    RCLCPP_INFO(node->get_logger(), 
                "Parallel finished after %d ticks with status: %s",
                tick_count, 
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : 
                (status == BT::NodeStatus::FAILURE) ? "FAILURE" : "RUNNING");
  }
  
  // ============================================
  // PART 3: REACTIVE SEQUENCE (another fresh factory)
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 3: Reactive Nodes ===");
    
    BT::BehaviorTreeFactory factory3;
    factory3.registerNodeType<BT_Tutorials::ActionA>("ActionA");
    factory3.registerNodeType<BT_Tutorials::ActionB>("ActionB");
    
    const std::string xml_reactive = R"(
      <root main_tree_to_execute="ReactiveExample">
        <BehaviorTree ID="ReactiveExample">
          <ReactiveSequence name="reactive">
            <ActionA/>
            <ActionB/>
          </ReactiveSequence>
        </BehaviorTree>
      </root>
    )";

    RCLCPP_INFO(node->get_logger(), 
                "ReactiveSequence re-evaluates all children each tick");
    RCLCPP_INFO(node->get_logger(), 
                "Watch ActionA get called multiple times:");

    auto tree_reactive = factory3.createTreeFromText(xml_reactive);
    
    for (int i = 0; i < 3; i++)
    {
      RCLCPP_INFO(node->get_logger(), "\n--- Tick %d ---", i + 1);
      tree_reactive.tickRoot();
      rclcpp::sleep_for(std::chrono::milliseconds(300));
    }
  }

  rclcpp::shutdown();
  return 0;
}
