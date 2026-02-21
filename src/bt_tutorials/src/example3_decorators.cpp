/**
 * @file example3_decorators.cpp
 * @brief Example 3: Decorator Nodes - ISOLATED VERSION
 */

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

namespace BT_Tutorials
{

class SimpleAction : public BT::SyncActionNode
{
public:
  SimpleAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config), counter_(0)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("message"),
      BT::OutputPort<int>("execution_count")
    };
  }

  BT::NodeStatus tick() override
  {
    counter_++;
    
    std::string msg = "default";
    getInput<std::string>("message", msg);
    
    RCLCPP_INFO(rclcpp::get_logger("SimpleAction"), 
                "[%s] Execution #%d: %s", 
                name().c_str(), counter_, msg.c_str());
    
    setOutput("execution_count", counter_);
    
    // Succeed on even attempts, fail on odd (for demonstration)
    return (counter_ % 2 == 0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  int counter_;
};

}  // namespace BT_Tutorials

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_example3");
  
  RCLCPP_INFO(node->get_logger(), 
              "Starting Behavior Tree Example 3: Decorator Nodes");

  // ============================================
  // PART 1: INVERTER Decorator
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 1: Inverter Decorator ===");
    RCLCPP_INFO(node->get_logger(), "Inverter changes SUCCESS ↔ FAILURE");
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT_Tutorials::SimpleAction>("SimpleAction");
    
    const std::string xml_inverter = R"(
      <root main_tree_to_execute="InverterExample">
        <BehaviorTree ID="InverterExample">
          <Sequence>
            <SimpleAction message="Without inverter" 
                         execution_count="{count1}"/>
            <Inverter>
              <SimpleAction message="With inverter" 
                           execution_count="{count2}"/>
            </Inverter>
          </Sequence>
        </BehaviorTree>
      </root>
    )";

    auto tree_inv = factory.createTreeFromText(xml_inverter);
    
    for (int i = 0; i < 2; i++)
    {
      RCLCPP_INFO(node->get_logger(), "\n--- Run %d ---", i + 1);
      auto status = tree_inv.tickRoot();
      RCLCPP_INFO(node->get_logger(), "Result: %s",
                  (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
    }
  }

  // ============================================
  // PART 2: RETRY Decorator
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 2: Retry Decorator ===");
    RCLCPP_INFO(node->get_logger(), "Retry: Execute child up to N times until success");
    
    BT::BehaviorTreeFactory factory2;
    factory2.registerNodeType<BT_Tutorials::SimpleAction>("SimpleAction");
    
    const std::string xml_retry = R"(
      <root main_tree_to_execute="RetryExample">
        <BehaviorTree ID="RetryExample">
          <RetryUntilSuccessful num_attempts="3">
            <SimpleAction message="Retry me" 
                         execution_count="{retry_count}"/>
          </RetryUntilSuccessful>
        </BehaviorTree>
      </root>
    )";

    RCLCPP_INFO(node->get_logger(), 
                "SimpleAction fails on odd attempts, succeeds on even");
    RCLCPP_INFO(node->get_logger(), 
                "Retry will try 3 times max (should succeed on 2nd attempt)");

    auto tree_retry = factory2.createTreeFromText(xml_retry);
    auto status_retry = tree_retry.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Retry result: %s",
                (status_retry == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 3: REPEAT Decorator
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 3: Repeat Decorator ===");
    RCLCPP_INFO(node->get_logger(), "Repeat: Execute child N times");
    
    BT::BehaviorTreeFactory factory3;
    factory3.registerNodeType<BT_Tutorials::SimpleAction>("SimpleAction");
    
    const std::string xml_repeat = R"(
      <root main_tree_to_execute="RepeatExample">
        <BehaviorTree ID="RepeatExample">
          <Repeat num_cycles="3">
            <SimpleAction message="Repeat me" 
                         execution_count="{repeat_count}"/>
          </Repeat>
        </BehaviorTree>
      </root>
    )";

    auto tree_repeat = factory3.createTreeFromText(xml_repeat);
    auto status_repeat = tree_repeat.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Repeat result: %s",
                (status_repeat == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 4: TIMEOUT Decorator
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 4: Timeout Decorator ===");
    RCLCPP_INFO(node->get_logger(), "Timeout: Fail if child takes too long");
    
    BT::BehaviorTreeFactory factory4;
    
    class SlowAction : public BT::AsyncActionNode
    {
    public:
      SlowAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::AsyncActionNode(name, config), completed_(false)
      {}

      static BT::PortsList providedPorts()
      {
        return { BT::InputPort<int>("delay_ms") };
      }

      BT::NodeStatus tick() override
      {
        int delay = 500;
        getInput<int>("delay_ms", delay);
        
        RCLCPP_INFO(rclcpp::get_logger("SlowAction"), 
                    "Starting slow action (%d ms)", delay);
        
        future_ = std::async(std::launch::async, [this, delay]() {
          std::this_thread::sleep_for(std::chrono::milliseconds(delay));
          completed_ = true;
          RCLCPP_INFO(rclcpp::get_logger("SlowAction"), "Slow action completed");
        });
        
        return BT::NodeStatus::RUNNING;
      }

      void halt() override
      {
        RCLCPP_WARN(rclcpp::get_logger("SlowAction"), "SlowAction halted!");
        completed_ = true;
      }

      bool isCompleted() const { return completed_; }

    private:
      std::future<void> future_;
      bool completed_;
    };
    
    factory4.registerNodeType<SlowAction>("SlowAction");
    factory4.registerNodeType<BT_Tutorials::SimpleAction>("SimpleAction");
    
    const std::string xml_timeout = R"(
      <root main_tree_to_execute="TimeoutExample">
        <BehaviorTree ID="TimeoutExample">
          <Fallback>
            <Timeout msec="300">
              <SlowAction delay_ms="500"/>
            </Timeout>
            <SimpleAction message="Fallback action"/>
          </Fallback>
        </BehaviorTree>
      </root>
    )";

    RCLCPP_INFO(node->get_logger(), 
                "SlowAction takes 500ms, Timeout is 300ms -> will timeout");
    RCLCPP_INFO(node->get_logger(), 
                "Fallback will execute SimpleAction");

    auto tree_timeout = factory4.createTreeFromText(xml_timeout);
    
    auto status = BT::NodeStatus::RUNNING;
    int ticks = 0;
    while (status == BT::NodeStatus::RUNNING && ticks < 10)
    {
      status = tree_timeout.tickRoot();
      ticks++;
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(node->get_logger(), "Timeout result after %d ticks: %s",
                ticks,
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : 
                (status == BT::NodeStatus::FAILURE) ? "FAILURE" : "RUNNING");
  }

  // ============================================
  // PART 5: KeepRunningUntilFailure
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), 
                "\n=== PART 5: KeepRunningUntilFailure Decorator ===");
    
    BT::BehaviorTreeFactory factory5;
    factory5.registerNodeType<BT_Tutorials::SimpleAction>("SimpleAction");
    
    const std::string xml_keep = R"(
      <root main_tree_to_execute="KeepExample">
        <BehaviorTree ID="KeepExample">
          <KeepRunningUntilFailure>
            <SimpleAction message="Keep running me"/>
          </KeepRunningUntilFailure>
        </BehaviorTree>
      </root>
    )";

    RCLCPP_INFO(node->get_logger(), 
                "SimpleAction fails on odd attempts");
    RCLCPP_INFO(node->get_logger(), 
                "Will run twice (1st: SUCCESS, 2nd: FAILURE -> stop)");

    auto tree_keep = factory5.createTreeFromText(xml_keep);
    auto status_keep = tree_keep.tickRoot();
    RCLCPP_INFO(node->get_logger(), "KeepRunningUntilFailure result: %s",
                (status_keep == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  rclcpp::shutdown();
  return 0;
}
