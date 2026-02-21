/**
 * =============================================================================
 * Student Submission Action Client
 * =============================================================================
 * 
 * DEMONSTRATES: ROS 2 Action Client for long-running tasks
 * 
 * FEATURES:
 * - Sends action goals to server
 * - Receives real-time feedback during execution
 * - Handles goal cancellation
 * - Monitors action lifecycle (accepted, executing, completed)
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg student_submission_client <student_id> <assignment> <processing_time>
 * 
 * EXAMPLES:
 *   ros2 run ros2_learning_pkg student_submission_client "S001" "ROS2_Final" 8.0
 *   ros2 run ros2_learning_pkg student_submission_client "S002" "C++_Midterm" 5.0
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_learning_pkg/action/process_student_submission.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;
using ProcessStudentSubmission = ros2_learning_pkg::action::ProcessStudentSubmission;
using GoalHandle = rclcpp_action::ClientGoalHandle<ProcessStudentSubmission>;

/**
 * =============================================================================
 * CLASS: StudentSubmissionClient
 * =============================================================================
 */
class StudentSubmissionClient : public rclcpp::Node
{
public:
    StudentSubmissionClient() : Node("student_submission_client")
    {
        // Create action client
        action_client_ = rclcpp_action::create_client<ProcessStudentSubmission>(
            this, "process_student_submission");
        
        RCLCPP_INFO(this->get_logger(), "🚀 Student Submission Action Client Started");
        RCLCPP_INFO(this->get_logger(), "🎯 Action: /process_student_submission");
    }

    /**
     * =========================================================================
     * METHOD: send_goal
     * =========================================================================
     * Main method to send action goal and handle the entire lifecycle
     */
    void send_goal(const std::string& student_id, 
                   const std::string& assignment_name,
                   float processing_time)
    {
        RCLCPP_INFO(this->get_logger(), "📤 Preparing to send action goal...");
        RCLCPP_INFO(this->get_logger(), "   Student: %s", student_id.c_str());
        RCLCPP_INFO(this->get_logger(), "   Assignment: %s", assignment_name.c_str());
        RCLCPP_INFO(this->get_logger(), "   Processing Time: %.1f seconds", processing_time);

        // Wait for action server to be available
        if (!action_client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Action server not available after 5 seconds");
            RCLCPP_INFO(this->get_logger(), "   Please start the server: ros2 run ros2_learning_pkg student_submission_server");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "✅ Action server found!");

        // Create goal message
        auto goal_msg = ProcessStudentSubmission::Goal();
        goal_msg.student_id = student_id;
        goal_msg.assignment_name = assignment_name;
        goal_msg.submission_content = "Student submission content for " + assignment_name;
        goal_msg.processing_time_seconds = processing_time;

        // Set up goal options
        auto send_goal_options = rclcpp_action::Client<ProcessStudentSubmission>::SendGoalOptions();
        
        // Set feedback callback
        send_goal_options.feedback_callback = 
            std::bind(&StudentSubmissionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        
        // Set result callback
        send_goal_options.result_callback = 
            std::bind(&StudentSubmissionClient::result_callback, this, std::placeholders::_1);
        
        // Set goal response callback
        send_goal_options.goal_response_callback =
            std::bind(&StudentSubmissionClient::goal_response_callback, this, std::placeholders::_1);

        RCLCPP_INFO(this->get_logger(), "🎯 Sending goal to action server...");

        // Send goal asynchronously
        auto future_goal_handle = action_client_->async_send_goal(goal_msg, send_goal_options);
        
        // Store the future for cancellation demo
        goal_handle_future_ = std::move(future_goal_handle);
        
        RCLCPP_INFO(this->get_logger(), "⏳ Goal sent! Waiting for processing to complete...");
        RCLCPP_INFO(this->get_logger(), "   Press Ctrl+C at any time to cancel the goal");
    }

    /**
     * =========================================================================
     * METHOD: cancel_goal
     * =========================================================================
     * Demonstrates goal cancellation (can be called from main)
     */
    void cancel_goal()
    {
        if (goal_handle_future_.valid() && 
            goal_handle_future_.wait_for(0s) == std::future_status::ready) {
            
            auto goal_handle = goal_handle_future_.get();
            if (goal_handle) {
                RCLCPP_INFO(this->get_logger(), "⏹️  Requesting goal cancellation...");
                auto future_cancel = action_client_->async_cancel_goal(goal_handle);
                
                if (future_cancel.wait_for(2s) == std::future_status::ready) {
                    RCLCPP_INFO(this->get_logger(), "✅ Cancellation request sent");
                }
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "No active goal to cancel");
        }
    }
        bool is_goal_completed() const {
        return goal_completed_;
    }

private:
    rclcpp_action::Client<ProcessStudentSubmission>::SharedPtr action_client_;
    std::shared_future<typename GoalHandle::SharedPtr> goal_handle_future_;
    std::atomic<bool> goal_completed_{false};

    /**
     * =========================================================================
     * CALLBACK: goal_response_callback
     * =========================================================================
     * Called when server responds to our goal (accepts or rejects)
     */
    void goal_response_callback(typename GoalHandle::SharedPtr goal_handle)
    {
       if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "❌ Goal was rejected by server");
    } 
        else {
            RCLCPP_INFO(this->get_logger(), "✅ Goal accepted by server, processing started!");
            RCLCPP_INFO(this->get_logger(), "   Goal ID: %s", 
                       rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
    } 
    }

    /**
     * =========================================================================
     * CALLBACK: feedback_callback
     * =========================================================================
     * Called periodically with progress updates from server
     */
    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const ProcessStudentSubmission::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "📊 FEEDBACK: %.1f%% - %s", 
                   feedback->progress_percentage, feedback->current_stage.c_str());
        
        // Show additional status if available
        if (!feedback->status_message.empty()) {
            RCLCPP_INFO(this->get_logger(), "   Status: %s", feedback->status_message.c_str());
        }
        
        // Warn if plagiarism detected
        if (feedback->is_plagiarism_detected) {
            RCLCPP_WARN(this->get_logger(), "   ⚠️  PLAGIARISM DETECTED!");
        }
        
        // Show progress bar visualization
        int bars = static_cast<int>(feedback->progress_percentage / 5);
        std::string progress_bar = "[" + std::string(bars, '=') + 
                                  std::string(20 - bars, ' ') + "]";
        RCLCPP_INFO(this->get_logger(), "   %s %.1f%%", progress_bar.c_str(), feedback->progress_percentage);
    }

    /**
     * =========================================================================
     * CALLBACK: result_callback
     * =========================================================================
     * Called when processing completes (success, cancellation, or failure)
     */
    void result_callback(const GoalHandle::WrappedResult & result)
    {
        goal_completed_ = true;
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "🎉 ACTION COMPLETED SUCCESSFULLY!");
                RCLCPP_INFO(this->get_logger(), "📝 FINAL RESULT:");
                RCLCPP_INFO(this->get_logger(), "   Grade: %.1f/100.0", result.result->final_grade);
                RCLCPP_INFO(this->get_logger(), "   Plagiarism Status: %s", result.result->plagiarism_status.c_str());
                RCLCPP_INFO(this->get_logger(), "   Feedback: %s", result.result->detailed_feedback.c_str());
                RCLCPP_INFO(this->get_logger(), "   Message: %s", result.result->completion_message.c_str());
                break;
                
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "⏹️  ACTION CANCELLED");
                RCLCPP_INFO(this->get_logger(), "   Message: %s", result.result->completion_message.c_str());
                break;
                
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "💥 ACTION ABORTED");
                RCLCPP_INFO(this->get_logger(), "   Message: %s", result.result->completion_message.c_str());
                break;
                
            default:
                RCLCPP_ERROR(this->get_logger(), "❓ UNKNOWN ACTION RESULT");
                break;
        }
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
    
    // Default values
    std::string student_id = "S001";
    std::string assignment = "ROS2_Assignment";
    float processing_time = 6.0f;
    
    // Parse command line arguments
    if (argc >= 2) student_id = argv[1];
    if (argc >= 3) assignment = argv[2];
    if (argc >= 4) processing_time = std::stof(argv[3]);
    
    auto client = std::make_shared<StudentSubmissionClient>();
    
    // Send the goal
    client->send_goal(student_id, assignment, processing_time);
    
    // Keep spinning to process callbacks
    RCLCPP_INFO(client->get_logger(), "🔄 Waiting for action to complete...");

    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && 
           std::chrono::steady_clock::now() - start_time < 60s) {
        rclcpp::spin_some(client);

        
        // Exit when goal is completed
        if (client->is_goal_completed()) {
            RCLCPP_INFO(client->get_logger(), "✅ Action completed, shutting down client");
            break;
        }

      std::this_thread::sleep_for(100ms);

    }
    // Handle timeout
    if (!client->is_goal_completed()) {
        RCLCPP_WARN(client->get_logger(), "⏰ Timeout reached, shutting down client");
    }  
    rclcpp::shutdown();
    return 0;
}
