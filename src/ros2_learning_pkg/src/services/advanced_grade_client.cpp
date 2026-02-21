/**
 * =============================================================================
 * Advanced Student Grade Service Client
 * =============================================================================
 * 
 * DEMONSTRATES: Multiple Service Calling Patterns & Timeout Strategies
 * 
 * PATTERNS IMPLEMENTED:
 * 1. Synchronous with Timeout
 * 2. Asynchronous with Future
 * 3. Asynchronous with Callback  
 * 4. Sequential Retry Pattern
 * 5. Parallel Multiple Requests
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg advanced_grade_client <pattern_id> [student_id] [assignment]
 * 
 * PATTERN IDs:
 *   1 = Synchronous with Timeout
 *   2 = Asynchronous with Future  
 *   3 = Asynchronous with Callback
 *   4 = Sequential Retry Pattern
 *   5 = Parallel Multiple Requests
 * 
 * EXAMPLES:
 *   ros2 run ros2_learning_pkg advanced_grade_client 1 "S001" "ROS2_Midterm"
 *   ros2 run ros2_learning_pkg advanced_grade_client 3
 */

// =============================================================================
// INCLUDES
// =============================================================================
#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_pkg/srv/student_grade.hpp"
#include <chrono>
#include <future>
#include <vector>
#include <memory>
#include <atomic>

using namespace std::chrono_literals;
using StudentGrade = ros2_learning_pkg::srv::StudentGrade;

/**
 * =============================================================================
 * CLASS: AdvancedGradeClient
 * =============================================================================
 * 
 * Implements multiple service calling patterns for educational purposes.
 * Each pattern demonstrates different approaches to handle service communication
 * with proper timeout and error handling strategies.
 */
class AdvancedGradeClient : public rclcpp::Node
{
public:
    AdvancedGradeClient() : Node("advanced_grade_client")
    {
        // Create service client - same as basic client
        client_ = this->create_client<StudentGrade>("get_student_grade");
        
        RCLCPP_INFO(this->get_logger(), "🚀 Advanced Grade Client Started");
        RCLCPP_INFO(this->get_logger(), "Service: /get_student_grade");
    }

    /**
     * =========================================================================
     * PATTERN 1: SYNCHRONOUS WITH TIMEOUT
     * =========================================================================
     * 
     * CHARACTERISTICS:
     * - Blocking call (thread waits for response)
     * - Simple, linear execution flow
     * - Easy error handling
     * - Wastes CPU cycles while waiting
     * - Good for simple scripts or sequential operations
     * 
     * USE CASE: When you MUST have the result before proceeding
     */
    void pattern_sync_with_timeout(const std::string& student_id, 
                                   const std::string& assignment_name)
    {
        RCLCPP_INFO(this->get_logger(), "🔄 PATTERN 1: Synchronous with Timeout");
        RCLCPP_INFO(this->get_logger(), "   Student: %s, Assignment: %s", 
                   student_id.c_str(), assignment_name.c_str());

        // Wait for service to be available (with timeout)
        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Service not available after 2 seconds");
            return;
        }

        // Create request
        auto request = std::make_shared<StudentGrade::Request>();
        request->student_id = student_id;
        request->assignment_name = assignment_name;

        auto start_time = std::chrono::steady_clock::now();
        
        // SYNC CALL: Send request and wait for response with timeout
        auto future = client_->async_send_request(request);
        
        // BLOCKING WAIT: Thread stops here until response or timeout
        auto wait_status = rclcpp::spin_until_future_complete(
            this->shared_from_this(),
            future,
            5s  // TIMEOUT: Maximum wait time
        );

        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        // Handle response based on wait status
        switch (wait_status) {
            case rclcpp::FutureReturnCode::SUCCESS:
                {
                    auto response = future.get();
                    RCLCPP_INFO(this->get_logger(), "✅ SUCCESS - Response received in %ld ms", duration.count());
                    RCLCPP_INFO(this->get_logger(), "   Grade: %.1f | Status: %s", 
                               response->grade, response->message.c_str());
                }
                break;
                
            case rclcpp::FutureReturnCode::TIMEOUT:
                RCLCPP_ERROR(this->get_logger(), "⏰ TIMEOUT - No response after 5 seconds");
                break;
                
            case rclcpp::FutureReturnCode::INTERRUPTED:
                RCLCPP_ERROR(this->get_logger(), "🚫 INTERRUPTED - Request was cancelled");
                break;
                
            default:
                RCLCPP_ERROR(this->get_logger(), "💥 UNKNOWN ERROR - Future return code: %d", static_cast<int>(wait_status));
        }
    }

    /**
     * =========================================================================
     * PATTERN 2: ASYNCHRONOUS WITH FUTURE
     * =========================================================================
     * 
     * CHARACTERISTICS:
     * - Non-blocking call
     * - Returns immediately with std::future
     * - Can check future status periodically
     * - More efficient CPU usage
     * - More complex error handling
     * 
     * USE CASE: When you want to do other work while waiting for response
     */
    void pattern_async_with_future(const std::string& student_id,
                                   const std::string& assignment_name) 
    {
        RCLCPP_INFO(this->get_logger(), "🔄 PATTERN 2: Asynchronous with Future");
        RCLCPP_INFO(this->get_logger(), "   Student: %s, Assignment: %s",
                   student_id.c_str(), assignment_name.c_str());

        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Service not available");
            return;
        }

        auto request = std::make_shared<StudentGrade::Request>();
        request->student_id = student_id;
        request->assignment_name = assignment_name;

        // ASYNC CALL: Send request without waiting
        auto future = client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "   Request sent, continuing immediately...");

        // SIMULATE DOING OTHER WORK
        for (int i = 1; i <= 3; i++) {
            RCLCPP_INFO(this->get_logger(), "   Doing other work... Step %d/3", i);
            std::this_thread::sleep_for(500ms);
            
            // NON-BLOCKING CHECK: See if response is ready
            auto status = future.wait_for(0ms);
            if (status == std::future_status::ready) {
                RCLCPP_INFO(this->get_logger(), "   Response arrived while working!");
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "   EARLY RESULT - Grade: %.1f", response->grade);
                return; // Exit early since we got response
            }
        }

        // FIXED: FINAL CHECK WITH PROPER SPINNING
        RCLCPP_INFO(this->get_logger(), "   Waiting for final response (max 4 seconds)...");
        auto start_wait = std::chrono::steady_clock::now();
        bool response_received = false;

        while (std::chrono::steady_clock::now() - start_wait < 4s && rclcpp::ok()) {
            // Check if response is ready (non-blocking)
            auto status = future.wait_for(100ms);
            
            if (status == std::future_status::ready) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "✅ FINAL RESULT RECEIVED!");
                RCLCPP_INFO(this->get_logger(), "   Grade: %.1f | Feedback: %s", 
                           response->grade, response->feedback.c_str());
                response_received = true;
                break;
            }
            
            // CRITICAL: Process ROS 2 callbacks to receive the response
            rclcpp::spin_some(this->shared_from_this());
            
            RCLCPP_INFO(this->get_logger(), "   Still waiting for response...");
            std::this_thread::sleep_for(100ms);
        }

        if (!response_received) {
            RCLCPP_WARN(this->get_logger(), "⌛ Final timeout - No response received");
        }
    }

    /**
     * =========================================================================
     * PATTERN 3: ASYNCHRONOUS WITH CALLBACK
     * =========================================================================
     * 
     * CHARACTERISTICS:
     * - Pure event-driven programming
     * - Callback function handles response
     * - No manual future management needed
     * - Most efficient for complex systems
     * - Response handling separated from request
     * 
     * USE CASE: Robot systems where you can't block the main thread
     */
    void pattern_async_with_callback(const std::string& student_id,
                                     const std::string& assignment_name)
    {
        RCLCPP_INFO(this->get_logger(), "🔄 PATTERN 3: Asynchronous with Callback");
        RCLCPP_INFO(this->get_logger(), "   Student: %s, Assignment: %s",
                   student_id.c_str(), assignment_name.c_str());

        if (!client_->wait_for_service(2s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Service not available");
            return;
        }

        auto request = std::make_shared<StudentGrade::Request>();
        request->student_id = student_id;
        request->assignment_name = assignment_name;

        // Reset the callback flag
        callback_executed_ = false;
        current_student_id_ = student_id;
        
        // ASYNC CALL WITH CALLBACK: Response handled by lambda function
        auto future = client_->async_send_request(
            request,
            [this](rclcpp::Client<StudentGrade>::SharedFuture future) {
                // This lambda runs WHEN response arrives (callback)
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "🎯 CALLBACK TRIGGERED for Student: %s", 
                           current_student_id_.c_str());
                RCLCPP_INFO(this->get_logger(), "   Grade: %.1f | Feedback: %s", 
                           response->grade, response->feedback.c_str());
                // SET THE FLAG to signal completion
                callback_executed_ = true;
            }
        );

        RCLCPP_INFO(this->get_logger(), "   Request sent with callback registered");
        RCLCPP_INFO(this->get_logger(), "   Waiting for callback (max 8 seconds)...");

        // FIXED: Wait for callback with proper exit condition
        auto start_time = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start_time < 8s && rclcpp::ok()) {
            // Process callbacks - THIS allows the callback to execute
            rclcpp::spin_some(this->shared_from_this());
            
            // Check if callback has been executed
            if (callback_executed_) {
                RCLCPP_INFO(this->get_logger(), "✅ Callback completed successfully!");
                return; // Exit the method
            }
            
            RCLCPP_INFO(this->get_logger(), "   Waiting for callback...");
            std::this_thread::sleep_for(300ms);
        }
        
        if (!callback_executed_) {
            RCLCPP_WARN(this->get_logger(), "⚠️  Callback wait timeout");
        }
    }

    /**
     * =========================================================================
     * PATTERN 4: SEQUENTIAL RETRY PATTERN
     * =========================================================================
     * 
     * CHARACTERISTICS:
     * - Automatic retry on failure
     * - Exponential backoff between retries
     * - Maximum retry limit
     * - Robust against temporary failures
     * 
     * USE CASE: Unreliable networks or temporary service unavailability
     */
    void pattern_sequential_retry(const std::string& student_id,
                                  const std::string& assignment_name)
    {
        RCLCPP_INFO(this->get_logger(), "🔄 PATTERN 4: Sequential Retry Pattern");
        
        const int max_retries = 3;
        const auto initial_timeout = 2s;
        
        for (int attempt = 1; attempt <= max_retries; attempt++) {
            RCLCPP_INFO(this->get_logger(), "   Attempt %d/%d - Student: %s", 
                       attempt, max_retries, student_id.c_str());

            if (!client_->wait_for_service(2s)) {
                RCLCPP_WARN(this->get_logger(), "   Service unavailable, retrying...");
                std::this_thread::sleep_for(initial_timeout * attempt); // Exponential backoff
                continue;
            }

            auto request = std::make_shared<StudentGrade::Request>();
            request->student_id = student_id;
            request->assignment_name = assignment_name;

            auto future = client_->async_send_request(request);
            auto wait_status = rclcpp::spin_until_future_complete(
                this->shared_from_this(), future, 4s);

            if (wait_status == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = future.get();
                RCLCPP_INFO(this->get_logger(), "✅ SUCCESS on attempt %d", attempt);
                RCLCPP_INFO(this->get_logger(), "   Grade: %.1f", response->grade);
                return; // Success - exit retry loop
            } else {
                RCLCPP_WARN(this->get_logger(), "   Attempt %d failed, retrying...", attempt);
                std::this_thread::sleep_for(initial_timeout * attempt);
            }
        }

        RCLCPP_ERROR(this->get_logger(), "💥 ALL %d ATTEMPTS FAILED", max_retries);
    }

    /**
     * =========================================================================
     * PATTERN 5: PARALLEL MULTIPLE REQUESTS
     * =========================================================================
     * 
     * CHARACTERISTICS:
     * - Send multiple requests simultaneously
     * - Wait for all responses
     * - Handle responses as they complete
     * - Maximum parallelism
     * 
     * USE CASE: Batch processing or querying multiple data sources
     */
    void pattern_parallel_requests()
    {
        RCLCPP_INFO(this->get_logger(), "🔄 PATTERN 5: Parallel Multiple Requests");

        if (!client_->wait_for_service(3s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Service not available");
            return;
        }

        // Create multiple student requests
        std::vector<std::pair<std::string, std::string>> students = {
            {"S001", "ROS2_Assignment_1"},
            {"S002", "ROS2_Assignment_1"}, 
            {"S003", "ROS2_Assignment_1"},
            {"S004", "ROS2_Assignment_1"}
        };

        std::vector<std::shared_future<typename StudentGrade::Response::SharedPtr>> futures;

        // SEND ALL REQUESTS IN PARALLEL (non-blocking)
        for (const auto& [student_id, assignment] : students) {
            auto request = std::make_shared<StudentGrade::Request>();
            request->student_id = student_id;
            request->assignment_name = assignment;

            auto future = client_->async_send_request(request);
            futures.push_back(future.share()); // CORRECT: Remove .future
            
            RCLCPP_INFO(this->get_logger(), "   Sent request for: %s", student_id.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "   All %zu requests sent, waiting for responses...", 
                   futures.size());

        // COLLECT ALL RESPONSES
        int success_count = 0;
        for (size_t i = 0; i < futures.size(); i++) {
            auto wait_status = rclcpp::spin_until_future_complete(
                this->shared_from_this(), futures[i], 6s);

            if (wait_status == rclcpp::FutureReturnCode::SUCCESS) {
                auto response = futures[i].get();
                RCLCPP_INFO(this->get_logger(), "   ✅ %s: %.1f/100.0", 
                           students[i].first.c_str(), response->grade);
                success_count++;
            } else {
                RCLCPP_WARN(this->get_logger(), "   ❌ %s: No response", 
                           students[i].first.c_str());
            }
        }

        RCLCPP_INFO(this->get_logger(), "📊 BATCH RESULTS: %d/%zu successful", 
                   success_count, futures.size());
    }

private:
    rclcpp::Client<StudentGrade>::SharedPtr client_;
    std::string current_student_id_; // For callback context
    std::atomic<bool> callback_executed_{false}; // Signal for callback completion
};

/**
 * =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto client = std::make_shared<AdvancedGradeClient>();
    
    // Default values
    std::string student_id = "S001";
    std::string assignment = "ROS2_Advanced_Patterns";
    int pattern_id = 1;
    
    // Parse command line arguments
    if (argc >= 2) pattern_id = std::stoi(argv[1]);
    if (argc >= 3) student_id = argv[2];
    if (argc >= 4) assignment = argv[3];
    
    RCLCPP_INFO(client->get_logger(), "🎯 Testing Pattern %d", pattern_id);
    
    // Execute selected pattern
    switch (pattern_id) {
        case 1:
            client->pattern_sync_with_timeout(student_id, assignment);
            break;
        case 2:
            client->pattern_async_with_future(student_id, assignment);
            break;
        case 3:
            client->pattern_async_with_callback(student_id, assignment);
            break;
        case 4:
            client->pattern_sequential_retry(student_id, assignment);
            break;
        case 5:
            client->pattern_parallel_requests();
            break;
        default:
            RCLCPP_ERROR(client->get_logger(), 
                        "Invalid pattern ID. Use 1-5");
            RCLCPP_INFO(client->get_logger(), 
                       "Patterns: 1=Sync, 2=Async Future, 3=Async Callback, 4=Retry, 5=Parallel");
    }
    
    // Keep node alive to receive callbacks (especially for pattern 3)
    //if (pattern_id == 3) {
        //RCLCPP_INFO(client->get_logger(), "Waiting for callback...");
        //rclcpp::spin(client);
    //}
    
    rclcpp::shutdown();
    return 0;
}
