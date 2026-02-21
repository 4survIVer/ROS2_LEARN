/**
 * =============================================================================
 * Load Balancing Service Client
 * =============================================================================
 * 
 * DEMONSTRATES: Load balancing across multiple service instances
 * 
 * STRATEGIES:
 * - Round Robin: Cycle through servers sequentially
 * - Random: Randomly select a server
 * - Least Connections: Choose server with fewest active requests
 * 
 * USE CASE: Distributing workload across multiple service instances
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg load_balancing_client <strategy>
 * 
 * STRATEGIES: round_robin, random, least_connections
 */

#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_pkg/srv/student_grade.hpp"
#include <chrono>
#include <memory>
#include <random>
#include <vector>
#include <atomic>

using namespace std::chrono_literals;
using StudentGrade = ros2_learning_pkg::srv::StudentGrade;

/**
 * =============================================================================
 * LOAD BALANCING STRATEGIES
 * =============================================================================
 */
enum class LoadBalanceStrategy {
    ROUND_ROBIN,
    RANDOM,
    LEAST_CONNECTIONS
};

/**
 * =============================================================================
 * CLASS: ServiceEndpoint
 * =============================================================================
 * Represents one service instance
 */
struct ServiceEndpoint {
    std::string name;
    rclcpp::Client<StudentGrade>::SharedPtr client;
    std::atomic<int> active_requests{0};
    std::atomic<bool> healthy{true};
    
    ServiceEndpoint(const std::string& endpoint_name, rclcpp::Node* node)
        : name(endpoint_name)
    {
        client = node->create_client<StudentGrade>(endpoint_name);
    }
};

/**
 * =============================================================================
 * CLASS: LoadBalancingClient
 * =============================================================================
 */
class LoadBalancingClient : public rclcpp::Node
{
public:
    LoadBalancingClient(LoadBalanceStrategy strategy = LoadBalanceStrategy::ROUND_ROBIN) 
        : Node("load_balancing_client"), strategy_(strategy)
    {
        // Create multiple service endpoints (simulating multiple servers)
        endpoints_.push_back(std::make_shared<ServiceEndpoint>("/get_student_grade", this));
        endpoints_.push_back(std::make_shared<ServiceEndpoint>("/get_student_grade", this));
        endpoints_.push_back(std::make_shared<ServiceEndpoint>("/get_student_grade", this));
        
        // Give them unique names for logging purposes
        endpoints_[0]->name = "Server_1";
        endpoints_[1]->name = "Server_2"; 
        endpoints_[2]->name = "Server_3";

        // Initialize random number generator
        rng_.seed(std::random_device{}());
        
        // Test timer
        test_timer_ = this->create_wall_timer(
            2s, std::bind(&LoadBalancingClient::send_test_request, this));
        
        RCLCPP_INFO(this->get_logger(), "🚀 Load Balancing Client Started");
        RCLCPP_INFO(this->get_logger(), "📊 Strategy: %s", strategy_to_string(strategy_).c_str());
        RCLCPP_INFO(this->get_logger(), "🔧 Endpoints: %zu servers", endpoints_.size());
    }

    /**
     * =========================================================================
     * METHOD: send_request_load_balanced
     * =========================================================================
     * Main method that applies load balancing
     */
    bool send_request_load_balanced(const std::string& student_id,
                                   const std::string& assignment_name)
    {
        auto endpoint = select_endpoint();
        if (!endpoint) {
            RCLCPP_ERROR(this->get_logger(), "❌ No healthy endpoints available");
            return false;
        }
        
        return send_to_endpoint(endpoint, student_id, assignment_name);
    }

private:
    std::vector<std::shared_ptr<ServiceEndpoint>> endpoints_;
    LoadBalanceStrategy strategy_;
    std::atomic<size_t> round_robin_index_{0};
    std::mt19937 rng_;
    rclcpp::TimerBase::SharedPtr test_timer_;

    /**
     * =========================================================================
     * METHOD: select_endpoint
     * =========================================================================
     * Selects an endpoint based on the load balancing strategy
     */
    std::shared_ptr<ServiceEndpoint> select_endpoint()
    {
        // Filter healthy endpoints
        std::vector<std::shared_ptr<ServiceEndpoint>> healthy_endpoints;
        for (const auto& endpoint : endpoints_) {
            if (endpoint->healthy) {
                healthy_endpoints.push_back(endpoint);
            }
        }
        
        if (healthy_endpoints.empty()) {
            return nullptr;
        }
        
        switch (strategy_) {
            case LoadBalanceStrategy::ROUND_ROBIN:
                return round_robin_select(healthy_endpoints);
                
            case LoadBalanceStrategy::RANDOM:
                return random_select(healthy_endpoints);
                
            case LoadBalanceStrategy::LEAST_CONNECTIONS:
                return least_connections_select(healthy_endpoints);
                
            default:
                return round_robin_select(healthy_endpoints);
        }
    }

    /**
     * =========================================================================
     * METHOD: round_robin_select
     * =========================================================================
     * Round Robin selection strategy
     */
    std::shared_ptr<ServiceEndpoint> round_robin_select(
        const std::vector<std::shared_ptr<ServiceEndpoint>>& healthy_endpoints)
    {
        size_t index = round_robin_index_++ % healthy_endpoints.size();
        auto selected = healthy_endpoints[index];
        
        RCLCPP_DEBUG(this->get_logger(), "🔄 Round Robin selected: %s", 
                    selected->name.c_str());
        return selected;
    }

    /**
     * =========================================================================
     * METHOD: random_select
     * =========================================================================
     * Random selection strategy
     */
    std::shared_ptr<ServiceEndpoint> random_select(
        const std::vector<std::shared_ptr<ServiceEndpoint>>& healthy_endpoints)
    {
        std::uniform_int_distribution<size_t> dist(0, healthy_endpoints.size() - 1);
        size_t index = dist(rng_);
        auto selected = healthy_endpoints[index];
        
        RCLCPP_DEBUG(this->get_logger(), "🎲 Random selected: %s", 
                    selected->name.c_str());
        return selected;
    }

    /**
     * =========================================================================
     * METHOD: least_connections_select
     * =========================================================================
     * Least Connections selection strategy
     */
    std::shared_ptr<ServiceEndpoint> least_connections_select(
        const std::vector<std::shared_ptr<ServiceEndpoint>>& healthy_endpoints)
    {
        auto selected = healthy_endpoints[0];
        int min_connections = selected->active_requests;
        
        for (const auto& endpoint : healthy_endpoints) {
            if (endpoint->active_requests < min_connections) {
                selected = endpoint;
                min_connections = endpoint->active_requests;
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "📊 Least Connections selected: %s (%d active)", 
                    selected->name.c_str(), min_connections);
        return selected;
    }

    /**
     * =========================================================================
     * METHOD: send_to_endpoint
     * =========================================================================
     * Sends request to a specific endpoint
     */
    bool send_to_endpoint(std::shared_ptr<ServiceEndpoint> endpoint,
                         const std::string& student_id,
                         const std::string& assignment_name)
    {
        // Increment active requests
        endpoint->active_requests++;
        
        if (!endpoint->client->wait_for_service(500s)) {
            endpoint->healthy = false;
            endpoint->active_requests--;
            RCLCPP_DEBUG(this->get_logger(), "⚠️  Endpoint %s is not ready", 
                       endpoint->name.c_str());
            return false;
        }
        
        auto request = std::make_shared<StudentGrade::Request>();
        request->student_id = student_id;
        request->assignment_name = assignment_name;
        
        auto future = endpoint->client->async_send_request(request);
        auto status = future.wait_for(5s);
        
        // Decrement active requests
        endpoint->active_requests--;
        
        if (status == std::future_status::ready) {
            try {
                auto response = future.get();
                endpoint->healthy = true; // Mark as healthy on success
                
                RCLCPP_INFO(this->get_logger(), "✅ %s - Grade: %.1f", 
                           endpoint->name.c_str(), response->grade);
                return true;
            } catch (const std::exception& e) {
                endpoint->healthy = false;
                RCLCPP_ERROR(this->get_logger(), "💥 %s - Exception: %s", 
                           endpoint->name.c_str(), e.what());
                return false;
            }
        } else {
            endpoint->healthy = false;
            RCLCPP_ERROR(this->get_logger(), "⏰ %s - Timeout", endpoint->name.c_str());
            return false;
        }
    }

    /**
     * =========================================================================
     * METHOD: strategy_to_string
     * =========================================================================
     * Converts strategy enum to string
     */
    std::string strategy_to_string(LoadBalanceStrategy strategy)
    {
        switch (strategy) {
            case LoadBalanceStrategy::ROUND_ROBIN: return "Round Robin";
            case LoadBalanceStrategy::RANDOM: return "Random";
            case LoadBalanceStrategy::LEAST_CONNECTIONS: return "Least Connections";
            default: return "Unknown";
        }
    }

    /**
     * =========================================================================
     * METHOD: send_test_request
     * =========================================================================
     * Sends periodic test requests
     */
    void send_test_request()
    {
        static int request_count = 0;
        request_count++;
        
        RCLCPP_INFO(this->get_logger(), "--- Load Balanced Request #%d ---", request_count);
        
        send_request_load_balanced(
            "LB" + std::to_string(request_count), 
            "Load_Balancing_Test");
        
        // Log endpoint status
        RCLCPP_INFO(this->get_logger(), "📊 Endpoint Status:");
        for (const auto& endpoint : endpoints_) {
            RCLCPP_INFO(this->get_logger(), "   %s: %s (%d active)", 
                       endpoint->name.c_str(),
                       endpoint->healthy ? "🟢" : "🔴",
                       endpoint->active_requests.load());
        }
        
        RCLCPP_INFO(this->get_logger(), "----------------------------");
    }
};

/**
 * =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Parse strategy from command line
    LoadBalanceStrategy strategy = LoadBalanceStrategy::ROUND_ROBIN;
    if (argc >= 2) {
        std::string strategy_str = argv[1];
        if (strategy_str == "random") {
            strategy = LoadBalanceStrategy::RANDOM;
        } else if (strategy_str == "least_connections") {
            strategy = LoadBalanceStrategy::LEAST_CONNECTIONS;
        }
    }
    
    auto client = std::make_shared<LoadBalancingClient>(strategy);
    
    RCLCPP_INFO(client->get_logger(), "🔧 Load Balancing Pattern Demonstration");
    RCLCPP_INFO(client->get_logger(), "💡 Strategies available:");
    RCLCPP_INFO(client->get_logger(), "   round_robin, random, least_connections");
    
    rclcpp::spin(client);
    rclcpp::shutdown();
    
    return 0;
}
