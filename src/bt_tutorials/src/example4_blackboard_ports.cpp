/**
 * @file example4_blackboard_ports.cpp
 * @brief Example 4: Blackboard and Ports - Data Sharing between Nodes
 * COMPLETE WORKING VERSION
 */

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <map>

namespace BT_Tutorials
{

// ============================================================================
// NODE 1: Data Producer (writes to Blackboard)
// ============================================================================
class DataProducer : public BT::SyncActionNode
{
public:
  DataProducer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::OutputPort<int>("sensor_value"),
      BT::OutputPort<std::string>("location"),
      BT::OutputPort<double>("temperature"),
      BT::BidirectionalPort<int>("counter")
    };
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("DataProducer"), 
                "Producing data...");
    
    // Write values to blackboard
    setOutput("sensor_value", 100);
    setOutput("location", "kitchen");
    setOutput("temperature", 22.5);
    
    // Update counter
    int counter = 0;
    if (getInput("counter", counter).has_value())
    {
      RCLCPP_INFO(rclcpp::get_logger("DataProducer"), 
                  "Current counter: %d", counter);
      counter++;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("DataProducer"), 
                  "Counter not found, initializing to 1");
      counter = 1;
    }
    setOutput("counter", counter);
    
    RCLCPP_INFO(rclcpp::get_logger("DataProducer"), 
                "Data production complete. Counter: %d", counter);
    
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// NODE 2: Data Consumer (reads from Blackboard)
// ============================================================================
class DataConsumer : public BT::SyncActionNode
{
public:
  DataConsumer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<int>("sensor_value"),
      BT::InputPort<std::string>("location"),
      BT::InputPort<double>("temperature"),
      BT::InputPort<int>("counter")
    };
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(rclcpp::get_logger("DataConsumer"), 
                "Consuming data...");
    
    int sensor_val;
    std::string location;
    double temperature;
    int counter;
    
    // Get all inputs
    auto sensor_result = getInput("sensor_value", sensor_val);
    auto location_result = getInput("location", location);
    auto temp_result = getInput("temperature", temperature);
    auto counter_result = getInput("counter", counter);
    
    // Check results
    if (sensor_result.has_value() && location_result.has_value() && 
        temp_result.has_value() && counter_result.has_value())
    {
      RCLCPP_INFO(rclcpp::get_logger("DataConsumer"), 
                  "Successfully read all data!");
      RCLCPP_INFO(rclcpp::get_logger("DataConsumer"), 
                  "Sensor: %d, Location: %s, Temp: %.1f°C, Counter: %d", 
                  sensor_val, location.c_str(), temperature, counter);
      
      if (sensor_val > 50)
      {
        RCLCPP_INFO(rclcpp::get_logger("DataConsumer"), 
                    "Sensor value above threshold!");
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        RCLCPP_INFO(rclcpp::get_logger("DataConsumer"), 
                    "Sensor value below threshold");
        return BT::NodeStatus::FAILURE;
      }
    }
    else
    {
      // Show what's missing
      RCLCPP_ERROR(rclcpp::get_logger("DataConsumer"), "Missing data:");
      if (!sensor_result.has_value()) 
        RCLCPP_ERROR(rclcpp::get_logger("DataConsumer"), "  sensor_value: %s", 
                    sensor_result.error().c_str());
      if (!location_result.has_value()) 
        RCLCPP_ERROR(rclcpp::get_logger("DataConsumer"), "  location: %s", 
                    location_result.error().c_str());
      if (!temp_result.has_value()) 
        RCLCPP_ERROR(rclcpp::get_logger("DataConsumer"), "  temperature: %s", 
                    temp_result.error().c_str());
      if (!counter_result.has_value()) 
        RCLCPP_ERROR(rclcpp::get_logger("DataConsumer"), "  counter: %s", 
                    counter_result.error().c_str());
      
      return BT::NodeStatus::FAILURE;
    }
  }
};

// ============================================================================
// NODE 3: Data Transformer (reads, processes, writes)
// ============================================================================
class DataTransformer : public BT::SyncActionNode
{
public:
  DataTransformer(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("input_temp_c"),
      BT::OutputPort<double>("output_temp_f"),
      BT::BidirectionalPort<int>("processing_count")
    };
  }

  BT::NodeStatus tick() override
  {
    double temp_c;
    auto temp_result = getInput("input_temp_c", temp_c);
    
    if (!temp_result.has_value())
    {
      RCLCPP_ERROR(rclcpp::get_logger("DataTransformer"), 
                   "Missing input temperature: %s", temp_result.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    int count = 0;
    getInput("processing_count", count);
    count++;
    setOutput("processing_count", count);
    
    double temp_f = (temp_c * 9.0/5.0) + 32.0;
    setOutput("output_temp_f", temp_f);
    
    RCLCPP_INFO(rclcpp::get_logger("DataTransformer"), 
                "Transformed: %.1f°C → %.1f°F (Processed %d times)", 
                temp_c, temp_f, count);
    
    return BT::NodeStatus::SUCCESS;
  }
};

// ============================================================================
// NODE 4: Conditional Node for INT values
// ============================================================================
class CheckIntCondition : public BT::ConditionNode
{
public:
  CheckIntCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<int>("value"),
      BT::InputPort<int>("limit"),
      BT::OutputPort<std::string>("status_message")
    };
  }

  BT::NodeStatus tick() override
  {
    int value, limit;
    
    auto value_result = getInput("value", value);
    auto limit_result = getInput("limit", limit);
    
    if (!value_result.has_value() || !limit_result.has_value())
    {
      setOutput("status_message", "Missing inputs");
      RCLCPP_ERROR(rclcpp::get_logger("CheckIntCondition"), 
                   "Missing inputs. Value: %s, Limit: %s",
                   value_result.error().c_str(),
                   limit_result.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    if (value > limit)
    {
      std::string msg = "Value " + std::to_string(value) + 
                       " > Limit " + std::to_string(limit);
      setOutput("status_message", msg);
      RCLCPP_INFO(rclcpp::get_logger("CheckIntCondition"), "%s", msg.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      std::string msg = "Value " + std::to_string(value) + 
                       " <= Limit " + std::to_string(limit);
      setOutput("status_message", msg);
      RCLCPP_INFO(rclcpp::get_logger("CheckIntCondition"), "%s", msg.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

// ============================================================================
// NODE 5: Conditional Node for DOUBLE values
// ============================================================================
class CheckDoubleCondition : public BT::ConditionNode
{
public:
  CheckDoubleCondition(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("value"),
      BT::InputPort<double>("limit"),
      BT::OutputPort<std::string>("status_message")
    };
  }

  BT::NodeStatus tick() override
  {
    double value, limit;
    
    auto value_result = getInput("value", value);
    auto limit_result = getInput("limit", limit);
    
    if (!value_result.has_value() || !limit_result.has_value())
    {
      setOutput("status_message", "Missing inputs");
      RCLCPP_ERROR(rclcpp::get_logger("CheckDoubleCondition"), 
                   "Missing inputs. Value: %s, Limit: %s",
                   value_result.error().c_str(),
                   limit_result.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    
    if (value > limit)
    {
      std::string msg = "Value " + std::to_string(value) + 
                       " > Limit " + std::to_string(limit);
      setOutput("status_message", msg);
      RCLCPP_INFO(rclcpp::get_logger("CheckDoubleCondition"), "%s", msg.c_str());
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      std::string msg = "Value " + std::to_string(value) + 
                       " <= Limit " + std::to_string(limit);
      setOutput("status_message", msg);
      RCLCPP_INFO(rclcpp::get_logger("CheckDoubleCondition"), "%s", msg.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }
};

}  // namespace BT_Tutorials

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bt_example4");
  
  RCLCPP_INFO(node->get_logger(), 
              "Starting Behavior Tree Example 4: Blackboard and Ports");
  RCLCPP_INFO(node->get_logger(), 
              "==========================================================");

  // ============================================
  // PART 1: Basic Data Flow - WORKING VERSION
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 1: Basic Data Flow ===");
    RCLCPP_INFO(node->get_logger(), "Producer → Consumer data sharing");
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT_Tutorials::DataProducer>("DataProducer");
    factory.registerNodeType<BT_Tutorials::DataConsumer>("DataConsumer");
    
    // KEY FIX: Use explicit port connections in XML
    const std::string xml_data_flow = R"(
      <root main_tree_to_execute="DataFlow">
        <BehaviorTree ID="DataFlow">
          <Sequence>
            <!-- Producer writes to blackboard with specific keys -->
            <DataProducer sensor_value="{sv}" 
                         location="{loc}" 
                         temperature="{temp}" 
                         counter="{cnt}"/>
            
            <!-- Consumer reads from the SAME keys -->
            <DataConsumer sensor_value="{sv}" 
                         location="{loc}" 
                         temperature="{temp}" 
                         counter="{cnt}"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";

    auto tree = factory.createTreeFromText(xml_data_flow);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing data flow tree...");
    auto status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Data flow result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 2: Bidirectional Ports
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 2: Bidirectional Ports ===");
    RCLCPP_INFO(node->get_logger(), "Data transformation with state tracking");
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT_Tutorials::DataTransformer>("DataTransformer");
    factory.registerNodeType<BT_Tutorials::CheckIntCondition>("CheckIntCondition");
    
    const std::string xml_transform = R"(
      <root main_tree_to_execute="Transform">
        <BehaviorTree ID="Transform">
          <Sequence>
            <!-- Initialize values with specific keys -->
            <SetBlackboard value="25.0" output_key="temp_c_key"/>
            <SetBlackboard value="0" output_key="count_key"/>
            
            <!-- Transform 3 times -->
            <Repeat num_cycles="3">
              <DataTransformer input_temp_c="{temp_c_key}"
                              output_temp_f="{temp_f_key}"
                              processing_count="{count_key}"/>
            </Repeat>
            
            <!-- Check result (count is int) -->
            <CheckIntCondition value="{count_key}" limit="2" 
                             status_message="{check_result_key}"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";

    auto tree = factory.createTreeFromText(xml_transform);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing transformation tree...");
    auto status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Transformation result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 3: Complete Pipeline - WORKING VERSION
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 3: Complete Pipeline ===");
    RCLCPP_INFO(node->get_logger(), "All nodes working together");
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT_Tutorials::DataProducer>("DataProducer");
    factory.registerNodeType<BT_Tutorials::DataConsumer>("DataConsumer");
    factory.registerNodeType<BT_Tutorials::DataTransformer>("DataTransformer");
    factory.registerNodeType<BT_Tutorials::CheckIntCondition>("CheckIntCondition");
    factory.registerNodeType<BT_Tutorials::CheckDoubleCondition>("CheckDoubleCondition");
    
    const std::string xml_pipeline = R"(
      <root main_tree_to_execute="Pipeline">
        <BehaviorTree ID="Pipeline">
          <Sequence>
            <!-- Step 1: Produce data -->
            <DataProducer sensor_value="{sensor_val}" 
                         location="{loc}" 
                         temperature="{temp}" 
                         counter="{cnt}"/>
            
            <!-- Step 2: Check sensor value (int) -->
            <Fallback>
              <CheckIntCondition value="{sensor_val}" limit="80"
                               status_message="{check1_msg}"/>
              <Sequence>
                <!-- If check fails, adjust value and check again -->
                <SetBlackboard value="200" output_key="sensor_val"/>
                <CheckIntCondition value="{sensor_val}" limit="80"
                                 status_message="{check2_msg}"/>
              </Sequence>
            </Fallback>
            
            <!-- Step 3: Transform temperature -->
            <Sequence>
              <SetBlackboard value="30.0" output_key="input_temp_c"/>
              <DataTransformer input_temp_c="{input_temp_c}"
                              output_temp_f="{output_temp_f}"
                              processing_count="{proc_cnt}"/>
            </Sequence>
            
            <!-- Step 4: Final check (double) and consume -->
            <Sequence>
              <!-- Use CheckDoubleCondition for double values -->
              <CheckDoubleCondition value="{output_temp_f}" limit="85.0"
                                  status_message="{temp_check_msg}"/>
              <DataConsumer sensor_value="{sensor_val}" 
                           location="{loc}" 
                           temperature="{temp}" 
                           counter="{cnt}"/>
            </Sequence>
          </Sequence>
        </BehaviorTree>
      </root>
    )";

    auto tree = factory.createTreeFromText(xml_pipeline);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing pipeline tree...");
    auto status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Pipeline result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  // ============================================
  // PART 4: Complex Data Types - WORKING VERSION
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 4: Complex Data Types ===");
    RCLCPP_INFO(node->get_logger(), "Working with vectors and maps");
    
    class VectorDataNode : public BT::SyncActionNode
    {
    public:
      VectorDataNode(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
      {}

      static BT::PortsList providedPorts()
      {
        return { 
          BT::OutputPort<std::vector<int>>("readings"),
          BT::OutputPort<std::map<std::string, double>>("sensor_data")
        };
      }

      BT::NodeStatus tick() override
      {
        std::vector<int> readings = {10, 20, 30, 40, 50};
        setOutput("readings", readings);
        
        std::map<std::string, double> sensor_data = {
          {"temperature", 23.5},
          {"humidity", 60.0},
          {"pressure", 1013.2}
        };
        setOutput("sensor_data", sensor_data);
        
        RCLCPP_INFO(rclcpp::get_logger("VectorDataNode"), 
                    "Created vector with %ld elements and map with %ld entries",
                    (long)readings.size(), (long)sensor_data.size());
        
        return BT::NodeStatus::SUCCESS;
      }
    };
    
    class ProcessVectorData : public BT::SyncActionNode
    {
    public:
      ProcessVectorData(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
      {}

      static BT::PortsList providedPorts()
      {
        return { 
          BT::InputPort<std::vector<int>>("readings"),
          BT::InputPort<std::map<std::string, double>>("sensor_data")
        };
      }

      BT::NodeStatus tick() override
      {
        std::vector<int> readings;
        std::map<std::string, double> sensor_data;
        
        auto readings_result = getInput("readings", readings);
        auto sensor_result = getInput("sensor_data", sensor_data);
        
        if (!readings_result.has_value() || !sensor_result.has_value())
        {
          RCLCPP_ERROR(rclcpp::get_logger("ProcessVectorData"), 
                       "Failed to read data. Readings: %s, Sensor: %s",
                       readings_result.error().c_str(),
                       sensor_result.error().c_str());
          return BT::NodeStatus::FAILURE;
        }
        
        RCLCPP_INFO(rclcpp::get_logger("ProcessVectorData"), 
                    "Processing data...");
        
        // Process vector
        int sum = 0;
        for (int val : readings) sum += val;
        double avg = static_cast<double>(sum) / readings.size();
        RCLCPP_INFO(rclcpp::get_logger("ProcessVectorData"), 
                    "Average reading: %.2f", avg);
        
        // Process map
        RCLCPP_INFO(rclcpp::get_logger("ProcessVectorData"), "Sensor data:");
        for (const auto& entry : sensor_data)
        {
          RCLCPP_INFO(rclcpp::get_logger("ProcessVectorData"), 
                      "  %s: %.2f", entry.first.c_str(), entry.second);
        }
        
        return BT::NodeStatus::SUCCESS;
      }
    };
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<VectorDataNode>("VectorDataNode");
    factory.registerNodeType<ProcessVectorData>("ProcessVectorData");
    
    const std::string xml_vector = R"(
      <root main_tree_to_execute="VectorData">
        <BehaviorTree ID="VectorData">
          <Sequence>
            <VectorDataNode readings="{rdngs}" sensor_data="{snsr_data}"/>
            <ProcessVectorData readings="{rdngs}" sensor_data="{snsr_data}"/>
          </Sequence>
        </BehaviorTree>
      </root>
    )";

    auto tree = factory.createTreeFromText(xml_vector);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing complex data tree...");
    tree.tickRoot();
  }

  // ============================================
  // PART 5: Port Remapping - WORKING VERSION
  // ============================================
  {
    RCLCPP_INFO(node->get_logger(), "\n=== PART 5: Port Remapping ===");
    RCLCPP_INFO(node->get_logger(), "Connecting different data sources to same node");
    
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<BT_Tutorials::CheckIntCondition>("CheckIntCondition");
    
    const std::string xml_remap = R"(
      <root main_tree_to_execute="RemapExample">
        <BehaviorTree ID="RemapExample">
          <Sequence>
            <!-- Create multiple data values -->
            <SetBlackboard value="50" output_key="val1"/>
            <SetBlackboard value="75" output_key="val2"/>
            <SetBlackboard value="100" output_key="val3"/>
            
            <!-- Use same node with different inputs -->
            <Sequence>
              <CheckIntCondition value="{val1}" limit="60" 
                               status_message="{res1}"/>
              <CheckIntCondition value="{val2}" limit="60" 
                               status_message="{res2}"/>
              <CheckIntCondition value="{val3}" limit="60" 
                               status_message="{res3}"/>
            </Sequence>
          </Sequence>
        </BehaviorTree>
      </root>
    )";

    auto tree = factory.createTreeFromText(xml_remap);
    BT::StdCoutLogger logger(tree);
    
    RCLCPP_INFO(node->get_logger(), "Executing port remapping example...");
    auto status = tree.tickRoot();
    RCLCPP_INFO(node->get_logger(), "Remap example result: %s",
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
  }

  RCLCPP_INFO(node->get_logger(), 
              "\n==========================================================");
  RCLCPP_INFO(node->get_logger(), "Example 4 completed successfully!");
  
  rclcpp::shutdown();
  return 0;
}