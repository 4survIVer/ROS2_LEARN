/**
 * =============================================================================
 * Circuit Breaker Service Client
 * =============================================================================
 * 
 * DEMONSTRATES: Circuit Breaker pattern for fault tolerance
 * 
 * BEHAVIOR:
 * - CLOSED:   Normal operation, requests allowed
 * - OPEN:     Service failing, requests blocked (fast failure)
 * - HALF_OPEN: Testing if service recovered
 * 
 * USE CASE: Prevents overwhelming failing services and cascading failures
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg circuit_breaker_client
 */

#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_pkg/srv/student_grade.hpp"
#include <chrono>
#include <memory>
#include <atomic>

using namespace std::chrono_literals;
using StudentGrade = ros2_learning_pkg::srv::StudentGrade;

/**
 * =============================================================================
 * CIRCUIT BREAKER STATES
 * =============================================================================
 */
enum class CircuitState {
    CLOSED,     // Normal operation - requests flow through
    OPEN,       // Circuit open - fail fast, no requests
    HALF_OPEN   // Testing if service recovered
};

/**
 * =============================================================================
 * CLASS: CircuitBreakerClient
 * =============================================================================
 */
class CircuitBreakerClient : public rclcpp::Node
{
public:
    CircuitBreakerClient() : Node("circuit_breaker_client")
    {
        // Create service client
        client_ = this->create_client<StudentGrade>("get_student_grade");
        
        // Circuit breaker parameters
        failure_threshold_ = 3;      // Open circuit after 3 failures
        success_threshold_ = 2;      // Close circuit after 2 successes
        timeout_duration_ = 5s;      // Timeout for each request
        reset_timeout_ = 10s;        // How long to wait before testing recovery
        
        current_state_ = CircuitState::CLOSED;
        failure_count_ = 0;
        success_count_ = 0;
        
        // Timer for state management
        state_timer_ = this->create_wall_timer(
            1s, std::bind(&CircuitBreakerClient::state_manager, this));
        
        // Timer for test requests
        test_timer_ = this->create_wall_timer(
            3s, std::bind(&CircuitBreakerClient::send_test_request, this));
        
        RCLCPP_INFO(this->get_logger(), "🚀 Circuit Breaker Client Started");
        RCLCPP_INFO(this->get_logger(), "⚡ Initial State: CLOSED");
        RCLCPP_INFO(this->get_logger(), "📊 Failure Threshold: %d", failure_threshold_);
        RCLCPP_INFO(this->get_logger(), "📊 Success Threshold: %d", success_threshold_);
    }

private:
    rclcpp::Client<StudentGrade>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    
    // Circuit breaker state
    std::atomic<CircuitState> current_state_;
    std::atomic<int> failure_count_;
    std::atomic<int> success_count_;
    std::chrono::steady_clock::time_point last_failure_time_;
    
    // Configuration
    int failure_threshold_;
    int success_threshold_;
    std::chrono::seconds timeout_duration_;
    std::chrono::seconds reset_timeout_;

    /**
     * =========================================================================
     * METHOD: send_request_with_circuit_breaker
     * =========================================================================
     * Main method that applies circuit breaker logic
     */
    bool send_request_with_circuit_breaker(const std::string& student_id, 
                                          const std::string& assignment_name)
    {
        // Check circuit state
        switch (current_state_) {
            case CircuitState::OPEN:
                RCLCPP_WARN(this->get_logger(), "⛔ CIRCUIT OPEN - Request blocked");
                return false;
                
            case CircuitState::HALF_OPEN:
                RCLCPP_INFO(this->get_logger(), "🟡 CIRCUIT HALF-OPEN - Testing service");
                break;
                
            case CircuitState::CLOSED:
                RCLCPP_DEBUG(this->get_logger(), "🟢 CIRCUIT CLOSED - Request allowed");
                break;
        }
        
        // Actually send the request
        return send_actual_request(student_id, assignment_name);
    }

    /**
     * =========================================================================
     * METHOD: send_actual_request
     * =========================================================================
     * Sends the actual service request and updates circuit state
     */
    bool send_actual_request(const std::string& student_id,
                            const std::string& assignment_name)
    {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_DEBUG(this->get_logger(), "Service not ready yet");
            record_failure();
            return false;
        }
        
        auto request = std::make_shared<StudentGrade::Request>();
        request->student_id = student_id;
        request->assignment_name = assignment_name;
        
        auto future = client_->async_send_request(request);
        RCLCPP_DEBUG(this->get_logger(), "Sending request to service...");
        auto status = future.wait_for(timeout_duration_);
        
        if (status == std::future_status::ready) {
            try {
                auto response = future.get();
                if (response->success) {
                    record_success();
                    RCLCPP_INFO(this->get_logger(), "✅ Request successful - Grade: %.1f", 
                               response->grade);
                    return true;
                } else {
                    record_failure();
                    RCLCPP_WARN(this->get_logger(), "⚠️  Service error: %s", 
                               response->message.c_str());
                    return false;
                }
            } catch (const std::exception& e) {
                record_failure();
                RCLCPP_ERROR(this->get_logger(), "💥 Exception: %s", e.what());
                return false;
            }
        } else {
            record_failure();
            RCLCPP_ERROR(this->get_logger(), "⏰ Request timeout");
            return false;
        }
    }

    /**
     * =========================================================================
     * METHOD: record_success
     * =========================================================================
     * Updates circuit state on successful request
     */
    void record_success()
    {
        failure_count_ = 0;
        auto state = current_state_.load();  // Get current state
    
        RCLCPP_INFO(this->get_logger(), "🎯 SUCCESS recorded");
    
        if (state == CircuitState::HALF_OPEN) {
            int current_success = ++success_count_;
            RCLCPP_INFO(this->get_logger(), "📈 Success count: %d/%d", 
                   current_success, success_threshold_);
        
            if (current_success >= success_threshold_) {
                current_state_ = CircuitState::CLOSED;
                success_count_ = 0;
                RCLCPP_INFO(this->get_logger(), "🟢 CIRCUIT CLOSED - Service recovered!");
            }
        }
    }

    /**
     * =========================================================================
     * METHOD: record_failure
     * =========================================================================
     * Updates circuit state on failed request
     */
    void record_failure()
    {
        int current_failures = ++failure_count_;
        last_failure_time_ = std::chrono::steady_clock::now();
    
        RCLCPP_WARN(this->get_logger(), "📉 Failure count: %d/%d", 
                current_failures, failure_threshold_);
    
        auto state = current_state_.load();  // Get current state
        if (current_failures >= failure_threshold_ && state == CircuitState::CLOSED) {
            current_state_ = CircuitState::OPEN;
            RCLCPP_ERROR(this->get_logger(), "🔴 CIRCUIT OPEN - Too many failures!");
        }
    
        if (state == CircuitState::HALF_OPEN) {
            success_count_ = 0; // Reset success count on failure
            RCLCPP_INFO(this->get_logger(), "🔄 Reset success count in HALF_OPEN state");
        }
    }

    /**
     * =========================================================================
     * METHOD: state_manager
     * =========================================================================
     * Manages circuit state transitions
     */
    void state_manager()
    {
        auto now = std::chrono::steady_clock::now();
        
        // Transition from OPEN to HALF_OPEN after reset timeout
        if (current_state_ == CircuitState::OPEN) {
            auto time_since_failure = now - last_failure_time_;
            
            if (time_since_failure >= reset_timeout_) {
                current_state_ = CircuitState::HALF_OPEN;
                RCLCPP_INFO(this->get_logger(), "🟡 CIRCUIT HALF-OPEN - Testing service recovery");
            }
        }
        
        // Log current state periodically
        static int log_counter = 0;
        if (++log_counter >= 10) { // Log every 10 seconds
            log_counter = 0;
            std::string state_str;
            switch (current_state_) {
                case CircuitState::CLOSED: state_str = "CLOSED"; break;
                case CircuitState::OPEN: state_str = "OPEN"; break;
                case CircuitState::HALF_OPEN: state_str = "HALF_OPEN"; break;
            }
            RCLCPP_INFO(this->get_logger(), "📊 Circuit State: %s, Failures: %d, Successes: %d",
                       state_str.c_str(), failure_count_.load(), success_count_.load());
        }
    }

    /**
     * =========================================================================
     * METHOD: send_test_request
     * =========================================================================
     * Sends periodic test requests to demonstrate circuit breaker
     */
    void send_test_request()
    {
        static int request_count = 0;
        request_count++;
        
        RCLCPP_INFO(this->get_logger(), "--- Test Request #%d ---", request_count);
        
        bool success = send_request_with_circuit_breaker(
            "S" + std::to_string(request_count), 
            "Circuit_Breaker_Test");
        
        if (success) {
            RCLCPP_INFO(this->get_logger(), "🎯 Test request completed successfully");
        } else {
            RCLCPP_INFO(this->get_logger(), "💥 Test request failed or blocked");
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
    
    auto client = std::make_shared<CircuitBreakerClient>();
    
    RCLCPP_INFO(client->get_logger(), "🔧 Circuit Breaker Pattern Demonstration");
    RCLCPP_INFO(client->get_logger(), "💡 To test:");
    RCLCPP_INFO(client->get_logger(), "   1. Start server to see CLOSED state");
    RCLCPP_INFO(client->get_logger(), "   2. Stop server to see OPEN state");
    RCLCPP_INFO(client->get_logger(), "   3. Restart server to see HALF_OPEN → CLOSED");
    
    rclcpp::spin(client);
    rclcpp::shutdown();
    
    return 0;
}
