/**
 * =============================================================================
 * Advanced Submission Action Client
 * =============================================================================
 * 
 * DEMONSTRATES: Testing advanced goal management features
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg advanced_submission_client <student_id> <processing_time>
 * 
 * EXAMPLES:
 *   # Normal priority (5 seconds)
 *   ros2 run ros2_learning_pkg advanced_submission_client "S001" 5.0
 *   
 *   # High priority (quick processing)
 *   ros2 run ros2_learning_pkg advanced_submission_client "S002" 1.5
 *   
 *   # Low priority (long processing)  
 *   ros2 run ros2_learning_pkg advanced_submission_client "S003" 12.0
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_learning_pkg/action/process_student_submission.hpp"
#include <chrono>
#include <memory>
#include <atomic>

using namespace std::chrono_literals;
using ProcessStudentSubmission = ros2_learning_pkg::action::ProcessStudentSubmission;
using GoalHandle = rclcpp_action::ClientGoalHandle<ProcessStudentSubmission>;

class AdvancedSubmissionClient : public rclcpp::Node
{
public:
    AdvancedSubmissionClient() : Node("advanced_submission_client")
    {
        client_ = rclcpp_action::create_client<ProcessStudentSubmission>(
            this, "advanced_process_submission");
        
        RCLCPP_INFO(this->get_logger(), "🚀 Advanced Submission Client Started");
    }

    void send_goal(const std::string& student_id, float processing_time)
    {
        RCLCPP_INFO(this->get_logger(), "📤 Sending goal - Student: %s, Time: %.1fs", 
                   student_id.c_str(), processing_time);
        
        if (!client_->wait_for_action_server(5s)) {
            RCLCPP_ERROR(this->get_logger(), "❌ Action server not available");
            return;
        }
        
        auto goal_msg = ProcessStudentSubmission::Goal();
        goal_msg.student_id = student_id;
        goal_msg.assignment_name = "Advanced_Management_Test";
        goal_msg.submission_content = "Testing goal management features";
        goal_msg.processing_time_seconds = processing_time;
        
        auto send_goal_options = rclcpp_action::Client<ProcessStudentSubmission>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&AdvancedSubmissionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = 
            std::bind(&AdvancedSubmissionClient::result_callback, this, std::placeholders::_1);
        send_goal_options.goal_response_callback =
            std::bind(&AdvancedSubmissionClient::goal_response_callback, this, std::placeholders::_1);
        
        client_->async_send_goal(goal_msg, send_goal_options);
        
        RCLCPP_INFO(this->get_logger(), "⏳ Goal sent to advanced server");
    }

private:
    rclcpp_action::Client<ProcessStudentSubmission>::SharedPtr client_;
    std::atomic<bool> goal_completed_{false};

    void goal_response_callback(typename GoalHandle::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "❌ Goal rejected");
        } else {
            RCLCPP_INFO(this->get_logger(), "✅ Goal accepted");
        }
    }

    void feedback_callback(
        GoalHandle::SharedPtr,
        const std::shared_ptr<const ProcessStudentSubmission::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "📊 Feedback: %.1f%% - %s", 
                   feedback->progress_percentage, feedback->current_stage.c_str());
    }

    void result_callback(const GoalHandle::WrappedResult & result)
    {
        goal_completed_ = true;
        
        switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "🎉 SUCCESS - Grade: %.1f", 
                            result.result->final_grade);
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "⏹️  CANCELLED");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "💥 ABORTED");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "❓ UNKNOWN RESULT CODE");
                break;
            }
        
        RCLCPP_INFO(this->get_logger(), "   Message: %s", result.result->completion_message.c_str());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::string student_id = "S001";
    float processing_time = 5.0f;
    
    if (argc >= 2) student_id = argv[1];
    if (argc >= 3) processing_time = std::stof(argv[2]);
    
    auto client = std::make_shared<AdvancedSubmissionClient>();
    client->send_goal(student_id, processing_time);
    
    // Wait for completion
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && 
           std::chrono::steady_clock::now() - start_time < 30s) {
        rclcpp::spin_some(client);
        std::this_thread::sleep_for(100ms);
    }
    
    rclcpp::shutdown();
    return 0;
}
