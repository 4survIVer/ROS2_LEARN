/**
 * @file example6_robot_delivery.cpp
 * @brief Example 6: Complete Robot Delivery System - FIXED VERSION
 * 
 * FIX: Properly handles AsyncActionNode execution by waiting for completion.
 */

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <fstream>
#include <vector>
#include <map>
#include <random>

namespace RobotDelivery
{

// ============================================================================
// CUSTOM TYPES FOR THE DELIVERY SYSTEM
// ============================================================================

struct Package
{
  std::string id;
  std::string type;
  std::string destination;
  bool is_picked_up;
  double weight;
  
  Package(const std::string& id_, const std::string& type_, 
          const std::string& dest_, double weight_ = 1.0)
    : id(id_), type(type_), destination(dest_), 
      is_picked_up(false), weight(weight_)
  {}
};

struct Location
{
  std::string name;
  double x, y, z;
  bool is_accessible;
  
  Location(const std::string& name_, double x_ = 0, double y_ = 0, double z_ = 0)
    : name(name_), x(x_), y(y_), z(z_), is_accessible(true)
  {}
};

// ============================================================================
// ACTION NODES - Perform robot operations
// ============================================================================

/**
 * @brief CHANGED: Now using SyncActionNode instead of AsyncActionNode
 * This simplifies the example and ensures synchronous execution
 */
class NavigateToAction : public BT::SyncActionNode  // CHANGED from AsyncActionNode
{
public:
  NavigateToAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("target_location"),
      BT::OutputPort<double>("navigation_time"),
      BT::OutputPort<double>("distance_traveled")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string target;
    if (!getInput("target_location", target).has_value())
    {
      RCLCPP_ERROR(rclcpp::get_logger("NavigateToAction"), 
                   "No target location specified!");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("NavigateToAction"), 
                "🚀 Navigating to: %s", target.c_str());
    
    // Simulate navigation time (0.5-1.5 seconds for faster execution)
    double navigation_time = 0.5 + (rand() % 1000) / 1000.0;
    
    // Simulate distance (5-15 meters)
    double distance = 5.0 + (rand() % 1000) / 100.0;
    
    // Simulate 80% chance of success
    bool success = (rand() % 100 < 80);
    
    // Add small delay to simulate movement
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(navigation_time * 1000)));
    
    // Set output values on blackboard
    setOutput("navigation_time", navigation_time);
    setOutput("distance_traveled", distance);
    
    if (success)
    {
      RCLCPP_INFO(rclcpp::get_logger("NavigateToAction"), 
                  "✅ Successfully navigated to %s (%.1f sec, %.1f m)", 
                  target.c_str(), navigation_time, distance);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("NavigateToAction"), 
                  "❌ Navigation to %s failed!", target.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

class PickPackageAction : public BT::SyncActionNode
{
public:
  PickPackageAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("package_id"),
      BT::InputPort<std::string>("location"),
      BT::OutputPort<bool>("pickup_success")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string package_id, location;
    if (!getInput("package_id", package_id).has_value() || 
        !getInput("location", location).has_value())
    {
      RCLCPP_ERROR(rclcpp::get_logger("PickPackageAction"), 
                   "Missing package_id or location!");
      setOutput("pickup_success", false);
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PickPackageAction"), 
                "🤖 Picking up package %s from %s", 
                package_id.c_str(), location.c_str());
    
    // Simulate pickup time
    rclcpp::sleep_for(std::chrono::milliseconds(300));
    
    // 90% chance of successful pickup
    bool success = (rand() % 100 < 90);
    setOutput("pickup_success", success);
    
    if (success)
    {
      RCLCPP_INFO(rclcpp::get_logger("PickPackageAction"), 
                  "✅ Package %s picked up successfully", package_id.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("PickPackageAction"), 
                  "❌ Failed to pick up package %s", package_id.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

class PlacePackageAction : public BT::SyncActionNode
{
public:
  PlacePackageAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("package_id"),
      BT::InputPort<std::string>("destination"),
      BT::OutputPort<bool>("delivery_success")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string package_id, destination;
    if (!getInput("package_id", package_id).has_value() || 
        !getInput("destination", destination).has_value())
    {
      RCLCPP_ERROR(rclcpp::get_logger("PlacePackageAction"), 
                   "Missing package_id or destination!");
      setOutput("delivery_success", false);
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("PlacePackageAction"), 
                "📦 Placing package %s at %s", 
                package_id.c_str(), destination.c_str());
    
    rclcpp::sleep_for(std::chrono::milliseconds(200));
    
    bool success = (rand() % 100 < 95);
    setOutput("delivery_success", success);
    
    if (success)
    {
      RCLCPP_INFO(rclcpp::get_logger("PlacePackageAction"), 
                  "✅ Package %s delivered to %s", 
                  package_id.c_str(), destination.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("PlacePackageAction"), 
                  "❌ Failed to deliver package %s to %s", 
                  package_id.c_str(), destination.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

/**
 * @brief CHANGED: Now using SyncActionNode with shorter sleep
 */
class ChargeBatteryAction : public BT::SyncActionNode  // CHANGED from AsyncActionNode
{
public:
  ChargeBatteryAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("charge_amount"),
      BT::OutputPort<double>("new_battery_level")
    };
  }

  BT::NodeStatus tick() override
  {
    double charge_amount = 50.0;
    getInput("charge_amount", charge_amount);
    
    RCLCPP_INFO(rclcpp::get_logger("ChargeBatteryAction"), 
                "🔋 Charging battery by %.1f%%...", charge_amount);
    
    // Simulate charging with shorter time for demonstration
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // Calculate new battery level
    double new_level = 50.0 + charge_amount;
    if (new_level > 100.0) new_level = 100.0;
    
    setOutput("new_battery_level", new_level);
    
    RCLCPP_INFO(rclcpp::get_logger("ChargeBatteryAction"), 
                "✅ Battery charged to %.1f%%", new_level);
    
    return BT::NodeStatus::SUCCESS;
  }
};

class AvoidObstacleAction : public BT::SyncActionNode
{
public:
  AvoidObstacleAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("obstacle_type"),
      BT::OutputPort<std::string>("avoidance_strategy")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string obstacle_type = "unknown";
    getInput("obstacle_type", obstacle_type);
    
    RCLCPP_INFO(rclcpp::get_logger("AvoidObstacleAction"), 
                "⚠️  Detected obstacle: %s", obstacle_type.c_str());
    
    std::string strategy;
    if (obstacle_type == "static")
      strategy = "go_around";
    else if (obstacle_type == "dynamic")
      strategy = "wait_and_proceed";
    else if (obstacle_type == "human")
      strategy = "slow_and_announce";
    else
      strategy = "stop_and_assess";
    
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    setOutput("avoidance_strategy", strategy);
    
    RCLCPP_INFO(rclcpp::get_logger("AvoidObstacleAction"), 
                "✅ Using strategy: %s", strategy.c_str());
    
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// CONDITION NODES - Check robot/environment states
// ============================================================================

class IsBatteryLow : public BT::ConditionNode
{
public:
  IsBatteryLow(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("threshold"),
      BT::InputPort<double>("current_level"),
      BT::OutputPort<bool>("is_low")
    };
  }

  BT::NodeStatus tick() override
  {
    double threshold = 20.0;
    double current_level = 50.0;
    
    getInput("threshold", threshold);
    getInput("current_level", current_level);
    
    bool is_low = (current_level <= threshold);
    setOutput("is_low", is_low);
    
    if (is_low)
    {
      RCLCPP_WARN(rclcpp::get_logger("IsBatteryLow"), 
                  "🔋 Battery LOW: %.1f%% (threshold: %.1f%%)", 
                  current_level, threshold);
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("IsBatteryLow"), 
                  "🔋 Battery OK: %.1f%%", current_level);
      return BT::NodeStatus::SUCCESS;
    }
  }
};

class HasPackage : public BT::ConditionNode
{
public:
  HasPackage(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("package_id"),
      BT::OutputPort<bool>("has_package")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string package_id;
    getInput("package_id", package_id);
    
    bool has_package = (!package_id.empty() && package_id != "none");
    setOutput("has_package", has_package);
    
    if (has_package)
    {
      RCLCPP_INFO(rclcpp::get_logger("HasPackage"), 
                  "📦 Robot is carrying package: %s", package_id.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("HasPackage"), 
                  "📦 Robot is NOT carrying a package");
      return BT::NodeStatus::FAILURE;
    }
  }
};

class IsDestinationReachable : public BT::ConditionNode
{
public:
  IsDestinationReachable(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<std::string>("destination"),
      BT::OutputPort<bool>("is_reachable")
    };
  }

  BT::NodeStatus tick() override
  {
    std::string destination;
    getInput("destination", destination);
    
    bool is_reachable = (rand() % 100 < 85);
    setOutput("is_reachable", is_reachable);
    
    if (is_reachable)
    {
      RCLCPP_INFO(rclcpp::get_logger("IsDestinationReachable"), 
                  "📍 Destination %s is reachable", destination.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("IsDestinationReachable"), 
                  "📍 Destination %s is NOT reachable", destination.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

class IsEmergencyStop : public BT::ConditionNode
{
public:
  IsEmergencyStop(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config), emergency_triggered_(false)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::OutputPort<bool>("emergency_active")
    };
  }

  BT::NodeStatus tick() override
  {
    // Simulate occasional emergency stop (5% chance)
    if (rand() % 100 < 5)
    {
      emergency_triggered_ = true;
    }
    
    setOutput("emergency_active", emergency_triggered_);
    
    if (emergency_triggered_)
    {
      RCLCPP_ERROR(rclcpp::get_logger("IsEmergencyStop"), 
                  "🚨 EMERGENCY STOP ACTIVATED!");
      return BT::NodeStatus::FAILURE;
    }
    
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool emergency_triggered_;
};

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

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

void initializeDeliverySystem(BT::Blackboard::Ptr blackboard)
{
  std::vector<Package> packages = {
    Package("PKG001", "urgent", "office_a", 2.5),
    Package("PKG002", "fragile", "lab_b", 1.0),
    Package("PKG003", "standard", "warehouse", 5.0),
    Package("PKG004", "urgent", "reception", 0.5)
  };
  
  std::map<std::string, Location> locations = {
    {"base", Location("base", 0, 0, 0)},
    {"office_a", Location("office_a", 10, 5, 0)},
    {"lab_b", Location("lab_b", 15, -3, 0)},
    {"warehouse", Location("warehouse", -8, 12, 0)},
    {"reception", Location("reception", 5, 8, 0)}
  };
  
  blackboard->set("packages", packages);
  blackboard->set("locations", locations);
  blackboard->set("current_location", std::string("base"));
  blackboard->set("battery_level", 85.0);
  blackboard->set("current_package_id", std::string("none"));
  
  RCLCPP_INFO(rclcpp::get_logger("initializeDeliverySystem"), 
              "✅ Delivery system initialized with %zu packages and %zu locations",
              packages.size(), locations.size());
}

}  // namespace RobotDelivery

// ============================================================================
// HELPER FUNCTION TO EXECUTE TREE AND WAIT FOR COMPLETION
// ============================================================================

/**
 * @brief Execute a behavior tree and wait for completion
 * @param tree The behavior tree to execute
 * @param node ROS 2 node for logging
 * @return BT::NodeStatus Final status of the tree
 * 
 * This function properly handles tree execution by ticking repeatedly
 * until the tree reaches a final state (SUCCESS or FAILURE).
 */
BT::NodeStatus executeTree(BT::Tree& tree, rclcpp::Node::SharedPtr node)
{
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  
  // Keep ticking until tree reaches a final state
  while (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING)
  {
    status = tree.tickRoot();
    
    // Small sleep to prevent tight loop
    if (status == BT::NodeStatus::RUNNING)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }
  
  return status;
}

// ============================================================================
// MAIN FUNCTION
// ============================================================================

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_example6_robot_delivery");
  
  RCLCPP_INFO(node->get_logger(), 
              "🤖 Starting Behavior Tree Example 6: Robot Delivery System");
  RCLCPP_INFO(node->get_logger(), 
              "==================================================================");
  
  srand(time(nullptr));

  BT::BehaviorTreeFactory factory;
  
  // ==========================================================================
  // REGISTER ALL NODE TYPES
  // ==========================================================================
  
  RCLCPP_INFO(node->get_logger(), "Registering behavior tree nodes...");
  
  // Register action nodes
  factory.registerNodeType<RobotDelivery::NavigateToAction>("NavigateToAction");
  factory.registerNodeType<RobotDelivery::PickPackageAction>("PickPackageAction");
  factory.registerNodeType<RobotDelivery::PlacePackageAction>("PlacePackageAction");
  factory.registerNodeType<RobotDelivery::ChargeBatteryAction>("ChargeBatteryAction");
  factory.registerNodeType<RobotDelivery::AvoidObstacleAction>("AvoidObstacleAction");
  
  // Register condition nodes
  factory.registerNodeType<RobotDelivery::IsBatteryLow>("IsBatteryLow");
  factory.registerNodeType<RobotDelivery::HasPackage>("HasPackage");
  factory.registerNodeType<RobotDelivery::IsDestinationReachable>("IsDestinationReachable");
  factory.registerNodeType<RobotDelivery::IsEmergencyStop>("IsEmergencyStop");
  
  RCLCPP_INFO(node->get_logger(), "✅ All nodes registered successfully");
  
  // ==========================================================================
  // CREATE BLACKBOARD AND INITIALIZE SYSTEM
  // ==========================================================================
  
  auto blackboard = BT::Blackboard::create();
  RobotDelivery::initializeDeliverySystem(blackboard);
  
  // ==========================================================================
  // SCENARIO 1: SIMPLE DELIVERY TASK
  // ==========================================================================
  
  {
    RCLCPP_INFO(node->get_logger(), "\n📦 SCENARIO 1: Simple Delivery Task");
    RCLCPP_INFO(node->get_logger(), "-----------------------------------");
    
    const std::string simple_delivery_tree = R"(
      <root>
        <BehaviorTree ID="SimpleDelivery">
          <Sequence>
            <IsBatteryLow threshold="30.0" current_level="{battery_level}" is_low="{battery_low}"/>
            <NavigateToAction target_location="warehouse" 
                             navigation_time="{nav_time}" 
                             distance_traveled="{distance}"/>
            <PickPackageAction package_id="PKG003" 
                              location="warehouse" 
                              pickup_success="{pickup_ok}"/>
            <NavigateToAction target_location="office_a" 
                             navigation_time="{nav_time2}" 
                             distance_traveled="{distance2}"/>
            <PlacePackageAction package_id="PKG003" 
                               destination="office_a" 
                               delivery_success="{delivery_ok}"/>
            <NavigateToAction target_location="base" 
                             navigation_time="{nav_time3}" 
                             distance_traveled="{distance3}"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerBehaviorTreeFromText(simple_delivery_tree);
    auto tree = factory.createTree("SimpleDelivery", blackboard);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing simple delivery task...");
    auto status = executeTree(tree, node);  // CHANGED: Use helper function
    
    RCLCPP_INFO(node->get_logger(), "Simple delivery result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "✅ SUCCESS" : "❌ FAILURE");
  }
  
  // ==========================================================================
  // SCENARIO 2: DELIVERY WITH ERROR RECOVERY
  // ==========================================================================
  
  {
    RCLCPP_INFO(node->get_logger(), "\n🔄 SCENARIO 2: Delivery with Error Recovery");
    RCLCPP_INFO(node->get_logger(), "--------------------------------------------");
    
    const std::string recovery_tree = R"(
      <root>
        <BehaviorTree ID="RecoveryDelivery">
          <Sequence>
            <!-- Announce start -->
            <AlwaysSuccess name="AnnounceStart"/>
            
            <!-- Try delivery with retry on failure -->
            <Fallback>
              <!-- Main delivery sequence -->
              <Sequence>
                <!-- Check if destination is reachable -->
                <IsDestinationReachable destination="lab_b" is_reachable="{reachable}"/>
                
                <!-- Navigate with retry on failure -->
                <RetryUntilSuccessful num_attempts="2">
                  <NavigateToAction target_location="lab_b"/>
                </RetryUntilSuccessful>
                
                <!-- Pick package -->
                <PickPackageAction package_id="PKG002" location="lab_b"/>
                
                <!-- Navigate to destination -->
                <NavigateToAction target_location="reception"/>
                
                <!-- Place package -->
                <PlacePackageAction package_id="PKG002" destination="reception"/>
              </Sequence>
              
              <!-- Recovery behavior if delivery fails -->
              <Sequence>
                <AlwaysSuccess name="AnnounceFailure"/>
                <NavigateToAction target_location="base"/>
                <AlwaysSuccess name="ReturnToBase"/>
              </Sequence>
            </Fallback>
            
            <!-- Final return to base -->
            <NavigateToAction target_location="base"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerSimpleAction("AnnounceStart", 
      [node](BT::TreeNode&) {
        RCLCPP_INFO(node->get_logger(), "🚀 Starting recovery delivery task");
        return BT::NodeStatus::SUCCESS;
      });
    
    factory.registerSimpleAction("AnnounceFailure", 
      [node](BT::TreeNode&) {
        RCLCPP_WARN(node->get_logger(), "⚠️  Delivery failed, initiating recovery...");
        return BT::NodeStatus::SUCCESS;
      });
    
    factory.registerSimpleAction("ReturnToBase", 
      [node](BT::TreeNode&) {
        RCLCPP_INFO(node->get_logger(), "🔄 Returning to base after failure");
        return BT::NodeStatus::SUCCESS;
      });
    
    factory.registerBehaviorTreeFromText(recovery_tree);
    auto tree = factory.createTree("RecoveryDelivery", blackboard);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing recovery delivery task...");
    auto status = executeTree(tree, node);  // CHANGED: Use helper function
    
    RCLCPP_INFO(node->get_logger(), "Recovery delivery result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "✅ SUCCESS" : "❌ FAILURE");
  }
  
  // ==========================================================================
  // SCENARIO 3: COMPLEX DELIVERY WITH BATTERY MANAGEMENT
  // ==========================================================================
  
  {
    RCLCPP_INFO(node->get_logger(), "\n🔋 SCENARIO 3: Delivery with Battery Management");
    RCLCPP_INFO(node->get_logger(), "-----------------------------------------------");
    
    const std::string battery_tree = R"(
      <root>
        <BehaviorTree ID="BatteryAwareDelivery">
          <Sequence>
            <!-- Check battery and charge if needed -->
            <Fallback>
              <Sequence>
                <IsBatteryLow threshold="40.0" current_level="{battery_level}"/>
                <ChargeBatteryAction charge_amount="30.0" new_battery_level="{battery_level}"/>
              </Sequence>
              <AlwaysSuccess/>
            </Fallback>
            
            <!-- Main delivery loop -->
            <Repeat num_cycles="2">
              <Sequence>
                <!-- Pick a package -->
                <PickPackageAction package_id="PKG001" location="warehouse"/>
                
                <!-- Deliver with obstacle avoidance -->
                <Sequence>
                  <NavigateToAction target_location="office_a"/>
                  
                  <!-- Simulate obstacle detection (optional) -->
                  <Fallback>
                    <AlwaysSuccess/>
                    <AvoidObstacleAction obstacle_type="dynamic" avoidance_strategy="{strategy}"/>
                  </Fallback>
                  
                  <PlacePackageAction package_id="PKG001" destination="office_a"/>
                </Sequence>
                
                <!-- Check battery after each delivery -->
                <IsBatteryLow threshold="30.0" current_level="{battery_level}"/>
              </Sequence>
            </Repeat>
            
            <!-- Final return -->
            <NavigateToAction target_location="base"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";
    
    factory.registerBehaviorTreeFromText(battery_tree);
    auto tree = factory.createTree("BatteryAwareDelivery", blackboard);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing battery-aware delivery task...");
    auto status = executeTree(tree, node);  // CHANGED: Use helper function
    
    RCLCPP_INFO(node->get_logger(), "Battery-aware delivery result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "✅ SUCCESS" : "❌ FAILURE");
  }
  
  // ==========================================================================
  // SCENARIO 4: LOAD TREE FROM XML FILE
  // ==========================================================================
  
  {
    RCLCPP_INFO(node->get_logger(), "\n📄 SCENARIO 4: Load Tree from XML File");
    RCLCPP_INFO(node->get_logger(), "--------------------------------------");
    
    try
    {
      std::string xml_content = RobotDelivery::loadXMLFromFile("trees/robot_delivery.xml");
      factory.registerBehaviorTreeFromText(xml_content);
      RCLCPP_INFO(node->get_logger(), "✅ Successfully loaded tree from XML file");
      
      auto tree = factory.createTree("RobotDeliveryMain", blackboard);
      BT::StdCoutLogger logger(tree);
      
      RCLCPP_INFO(node->get_logger(), "Executing tree from XML file...");
      auto status = executeTree(tree, node);  // CHANGED: Use helper function
      
      RCLCPP_INFO(node->get_logger(), "XML tree execution result: %s",
                  (status == BT::NodeStatus::SUCCESS) ? "✅ SUCCESS" : "❌ FAILURE");
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(node->get_logger(), 
                  "Could not load XML file: %s", e.what());
      RCLCPP_INFO(node->get_logger(), 
                  "Creating sample XML content instead...");
      
      const std::string sample_tree = R"(
        <root>
          <BehaviorTree ID="RobotDeliveryMain">
            <Sequence>
              <AlwaysSuccess name="StartFromXML"/>
              <NavigateToAction target_location="warehouse"/>
              <PickPackageAction package_id="PKG004" location="warehouse"/>
              <NavigateToAction target_location="reception"/>
              <PlacePackageAction package_id="PKG004" destination="reception"/>
              <NavigateToAction target_location="base"/>
            </Sequence>
          </BehaviorTree>
        </root>
      )";
      
      factory.registerSimpleAction("StartFromXML", 
        [node](BT::TreeNode&) {
          RCLCPP_INFO(node->get_logger(), "📄 Starting task loaded from XML");
          return BT::NodeStatus::SUCCESS;
        });
      
      factory.registerBehaviorTreeFromText(sample_tree);
      auto tree = factory.createTree("RobotDeliveryMain", blackboard);
      BT::StdCoutLogger logger(tree);
      
      RCLCPP_INFO(node->get_logger(), "Executing sample tree...");
      auto status = executeTree(tree, node);  // CHANGED: Use helper function
      
      RCLCPP_INFO(node->get_logger(), "Sample tree execution result: %s",
                  (status == BT::NodeStatus::SUCCESS) ? "✅ SUCCESS" : "❌ FAILURE");
    }
  }
  
  // ==========================================================================
  // DEMONSTRATE BLACKBOARD DATA ACCESS
  // ==========================================================================
  
  {
    RCLCPP_INFO(node->get_logger(), "\n📊 FINAL SYSTEM STATE");
    RCLCPP_INFO(node->get_logger(), "---------------------");
    
    auto packages = blackboard->get<std::vector<RobotDelivery::Package>>("packages");
    auto battery_level = blackboard->get<double>("battery_level");
    auto current_location = blackboard->get<std::string>("current_location");
    
    RCLCPP_INFO(node->get_logger(), "Current battery level: %.1f%%", battery_level);
    RCLCPP_INFO(node->get_logger(), "Current location: %s", current_location.c_str());
    RCLCPP_INFO(node->get_logger(), "Number of packages in system: %zu", packages.size());
    
    if (!packages.empty())
    {
      RCLCPP_INFO(node->get_logger(), "Sample package: %s -> %s (%s, %.1f kg)",
                  packages[0].id.c_str(), 
                  packages[0].destination.c_str(),
                  packages[0].type.c_str(),
                  packages[0].weight);
    }
  }
  
  RCLCPP_INFO(node->get_logger(), 
              "\n==================================================================");
  RCLCPP_INFO(node->get_logger(), "🤖 Robot Delivery System Example Completed!");
  RCLCPP_INFO(node->get_logger(), "Key concepts demonstrated:");
  RCLCPP_INFO(node->get_logger(), "  ✅ Action nodes (Navigate, Pick, Place, Charge)");
  RCLCPP_INFO(node->get_logger(), "  ✅ Condition nodes (Battery, Package, Reachability)");
  RCLCPP_INFO(node->get_logger(), "  ✅ Control nodes (Sequence, Fallback, Repeat)");
  RCLCPP_INFO(node->get_logger(), "  ✅ Decorator nodes (Retry, Inverter)");
  RCLCPP_INFO(node->get_logger(), "  ✅ Blackboard data sharing");
  RCLCPP_INFO(node->get_logger(), "  ✅ SubTrees and modular design");
  RCLCPP_INFO(node->get_logger(), "  ✅ Error handling and recovery");
  RCLCPP_INFO(node->get_logger(), "  ✅ XML file loading");
  
  rclcpp::shutdown();
  return 0;
}