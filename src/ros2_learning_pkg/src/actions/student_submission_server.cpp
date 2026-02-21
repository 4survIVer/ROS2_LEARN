/**
 * =============================================================================
 * Student Submission Action Server
 * =============================================================================
 * 
 * DEMONSTRATES: ROS 2 Actions for long-running tasks with progress feedback
 * 
 * FEATURES:
 * - Accepts action goals (student submissions to process)
 * - Provides real-time feedback during processing
 * - Handles goal cancellation
 * - Simulates multi-stage processing workflow
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg student_submission_server
 * 
 * ACTION: /process_student_submission
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_learning_pkg/action/process_student_submission.hpp"
#include <thread>
#include <chrono>
#include <atomic>

using namespace std::chrono_literals;
using ProcessStudentSubmission = ros2_learning_pkg::action::ProcessStudentSubmission;
using GoalHandle = rclcpp_action::ServerGoalHandle<ProcessStudentSubmission>;

/**
 * =============================================================================
 * CLASS: StudentSubmissionServer
 * =============================================================================
 */
class StudentSubmissionServer : public rclcpp::Node
{
public:
    StudentSubmissionServer() : Node("student_submission_server")
    {
        // Create action server
        action_server_ = rclcpp_action::create_server<ProcessStudentSubmission>(
            this,
            "process_student_submission",  // Action name
            std::bind(&StudentSubmissionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&StudentSubmissionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&StudentSubmissionServer::handle_accepted, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "🚀 Student Submission Action Server Started");
        RCLCPP_INFO(this->get_logger(), "📡 Action: /process_student_submission");
        RCLCPP_INFO(this->get_logger(), "⏳ Ready to process student submissions...");
    }

private:
    rclcpp_action::Server<ProcessStudentSubmission>::SharedPtr action_server_;

    /**
     * =========================================================================
     * METHOD: handle_goal
     * =========================================================================
     * Called when a new goal is received from a client
     * Decides whether to accept or reject the goal
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ProcessStudentSubmission::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "🎯 Received new goal request:");
        RCLCPP_INFO(this->get_logger(), "   Student: %s", goal->student_id.c_str());
        RCLCPP_INFO(this->get_logger(), "   Assignment: %s", goal->assignment_name.c_str());
        RCLCPP_INFO(this->get_logger(), "   Processing Time: %.1f seconds", goal->processing_time_seconds);
        
        // Validate the goal
        if (goal->student_id.empty() || goal->assignment_name.empty()) {
            RCLCPP_WARN(this->get_logger(), "   ❌ REJECTED: Empty student ID or assignment name");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        if (goal->processing_time_seconds <= 0) {
            RCLCPP_WARN(this->get_logger(), "   ❌ REJECTED: Invalid processing time");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(this->get_logger(), "   ✅ ACCEPTED: Goal validation passed");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * =========================================================================
     * METHOD: handle_cancel
     * =========================================================================
     * Called when client requests to cancel a running goal
     */
    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "⏹️  Received cancel request for goal");
        // In a real system, you'd stop the processing here
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * =========================================================================
     * METHOD: handle_accepted
     * =========================================================================
     * Called after goal is accepted - starts the actual processing
     */
    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
    {
        // Start processing in a separate thread to avoid blocking
        std::thread{std::bind(&StudentSubmissionServer::execute_processing, this, std::placeholders::_1), goal_handle}.detach();
    }

    /**
     * =========================================================================
     * METHOD: execute_processing
     * =========================================================================
     * Main processing logic - simulates long-running student submission processing
     * Provides feedback during execution and final result
     */
    void execute_processing(std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "🔨 Starting to process submission...");
        
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ProcessStudentSubmission::Result>();
        auto feedback = std::make_shared<ProcessStudentSubmission::Feedback>();
        
        const auto processing_time = goal->processing_time_seconds;
        const int total_steps = 10;  // Simulate 10 processing steps
        const auto step_duration = processing_time / total_steps;
        
        // SIMULATE PROCESSING WORKFLOW
        std::vector<std::string> processing_stages = {
            "Uploading submission...",
            "Syntax analysis...", 
            "Code quality check...",
            "Plagiarism detection...",
            "Test case execution...",
            "Performance benchmarking...",
            "Style guide verification...",
            "Documentation review...",
            "Final grading...",
            "Generating feedback..."
        };
        
        try {
            for (int step = 0; step < total_steps && rclcpp::ok(); step++) {
                // Check if goal was cancelled
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->completion_message = "Processing cancelled by user";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "❌ Processing cancelled");
                    return;
                }
                
                // Update feedback
                feedback->progress_percentage = (step * 100.0) / total_steps;
                feedback->current_stage = processing_stages[step];
                feedback->status_message = "Step " + std::to_string(step + 1) + "/" + std::to_string(total_steps);
                
                // Simulate plagiarism detection at step 3
                if (step == 3) {
                    // 20% chance of plagiarism detection
                    if (std::rand() % 100 < 20) {
                        feedback->is_plagiarism_detected = true;
                        feedback->status_message = "⚠️  Potential plagiarism detected!";
                    } else {
                        feedback->is_plagiarism_detected = false;
                    }
                }
                
                // Publish feedback
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "📊 Progress: %.1f%% - %s", 
                           feedback->progress_percentage, feedback->current_stage.c_str());
                
                // Simulate processing time for this step
                std::this_thread::sleep_for(std::chrono::duration<float>(step_duration));
            }
            
            // PROCESSING COMPLETE - Set final result
            // Generate deterministic grade based on student ID
            std::hash<std::string> hasher;
            float base_grade = (hasher(goal->student_id) % 71) + 30.0f; // 30-100 range
            
            result->final_grade = base_grade;
            result->detailed_feedback = "Excellent submission! Well structured code with good documentation.";
            result->plagiarism_status = feedback->is_plagiarism_detected ? "SUSPECTED" : "CLEAN";
            result->success = true;
            result->completion_message = "Submission processing completed successfully";
            
            // Mark goal as succeeded
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "✅ Processing completed for %s", goal->student_id.c_str());
            
        } catch (const std::exception& ex) {
            // Handle errors
            result->success = false;
            result->completion_message = "Error during processing: " + std::string(ex.what());
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "💥 Processing failed: %s", ex.what());
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
    
    // Seed random number generator for plagiarism simulation
    std::srand(std::time(nullptr));
    
    auto server = std::make_shared<StudentSubmissionServer>();
    rclcpp::spin(server);
    rclcpp::shutdown();
    
    return 0;
}
