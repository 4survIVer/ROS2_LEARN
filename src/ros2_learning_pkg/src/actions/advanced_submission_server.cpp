/**
 * =============================================================================
 * Advanced Student Submission Action Server
 * =============================================================================
 * 
 * DEMONSTRATES: Advanced Goal Management Patterns
 * 
 * FEATURES:
 * - Goal Preemption: New high-priority goals cancel current ones
 * - Goal Queuing: Multiple goals processed sequentially
 * - Priority System: Urgent goals jump the queue
 * - Goal State Management: Track goal lifecycle
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg advanced_submission_server
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_learning_pkg/action/process_student_submission.hpp"
#include <queue>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using ProcessStudentSubmission = ros2_learning_pkg::action::ProcessStudentSubmission;
using GoalHandle = rclcpp_action::ServerGoalHandle<ProcessStudentSubmission>;

/**
 * =============================================================================
 * GOAL PRIORITY LEVELS
 * =============================================================================
 */
enum class GoalPriority {
    LOW,        // Normal submissions
    NORMAL,     // Standard priority
    HIGH,       // Urgent submissions
    CRITICAL    // Emergency processing
};

/**
 * =============================================================================
 * STRUCT: QueuedGoal
 * =============================================================================
 * Wraps goal with additional management data
 */
struct QueuedGoal {
    std::shared_ptr<const ProcessStudentSubmission::Goal> goal;
    std::shared_ptr<GoalHandle> goal_handle;
    GoalPriority priority;
    std::chrono::steady_clock::time_point queue_time;
    
    QueuedGoal(std::shared_ptr<const ProcessStudentSubmission::Goal> g,
               std::shared_ptr<GoalHandle> gh,
               GoalPriority p = GoalPriority::NORMAL)
        : goal(g), goal_handle(gh), priority(p), 
          queue_time(std::chrono::steady_clock::now()) {}
    
    // For priority queue (higher priority first)
    bool operator<(const QueuedGoal& other) const {
        return static_cast<int>(priority) < static_cast<int>(other.priority);
    }
};

/**
 * =============================================================================
 * CLASS: AdvancedSubmissionServer
 * =============================================================================
 */
class AdvancedSubmissionServer : public rclcpp::Node
{
public:
    AdvancedSubmissionServer() : Node("advanced_submission_server")
    {
        // Create action server
        action_server_ = rclcpp_action::create_server<ProcessStudentSubmission>(
            this,
            "advanced_process_submission",
            std::bind(&AdvancedSubmissionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&AdvancedSubmissionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&AdvancedSubmissionServer::handle_accepted, this, std::placeholders::_1));
        
        // Worker thread for processing goals from queue
        worker_thread_ = std::thread(&AdvancedSubmissionServer::process_goal_queue, this);
        
        // Status timer
        status_timer_ = this->create_wall_timer(
            3s, std::bind(&AdvancedSubmissionServer::publish_status, this));
        
        RCLCPP_INFO(this->get_logger(), "🚀 Advanced Submission Server Started");
        RCLCPP_INFO(this->get_logger(), "🎯 Action: /advanced_process_submission");
        RCLCPP_INFO(this->get_logger(), "📊 Features: Preemption, Queuing, Priority System");
    }
    
    ~AdvancedSubmissionServer()
    {
        shutdown_requested_ = true;
        queue_cv_.notify_all();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

private:
    rclcpp_action::Server<ProcessStudentSubmission>::SharedPtr action_server_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::thread worker_thread_;
    
    // Goal management
    std::priority_queue<QueuedGoal> goal_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> shutdown_requested_{false};
    
    // Current execution state
    std::shared_ptr<GoalHandle> current_goal_handle_;
    std::atomic<bool> is_processing_{false};

    /**
     * =========================================================================
     * METHOD: handle_goal
     * =========================================================================
     * Validates and assigns priority to incoming goals
     */
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ProcessStudentSubmission::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "🎯 New goal received:");
        RCLCPP_INFO(this->get_logger(), "   Student: %s", goal->student_id.c_str());
        RCLCPP_INFO(this->get_logger(), "   Assignment: %s", goal->assignment_name.c_str());
        RCLCPP_INFO(this->get_logger(), "   Processing Time: %.1fs", goal->processing_time_seconds);
        
        // Validation
        if (goal->student_id.empty() || goal->assignment_name.empty()) {
            RCLCPP_WARN(this->get_logger(), "   ❌ REJECTED: Empty fields");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        if (goal->processing_time_seconds <= 0) {
            RCLCPP_WARN(this->get_logger(), "   ❌ REJECTED: Invalid processing time");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Assign priority based on processing time (shorter = higher priority)
        GoalPriority priority = GoalPriority::NORMAL;
        if (goal->processing_time_seconds <= 2.0) {
            priority = GoalPriority::HIGH;
            RCLCPP_INFO(this->get_logger(), "   🔥 HIGH PRIORITY: Quick processing");
        } else if (goal->processing_time_seconds >= 10.0) {
            priority = GoalPriority::LOW;
            RCLCPP_INFO(this->get_logger(), "   📉 LOW PRIORITY: Long processing");
        } else {
            RCLCPP_INFO(this->get_logger(), "   📊 NORMAL PRIORITY");
        }
        
        RCLCPP_INFO(this->get_logger(), "   ✅ ACCEPTED");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * =========================================================================
     * METHOD: handle_cancel
     * =========================================================================
     * Handles goal cancellation requests
     */
    rclcpp_action::CancelResponse handle_cancel(
        std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "⏹️  Cancel request for goal");
        
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // Check if this is the currently executing goal
        if (current_goal_handle_ && 
            current_goal_handle_->get_goal_id() == goal_handle->get_goal_id()) {
            RCLCPP_INFO(this->get_logger(), "   Cancelling currently executing goal");
            return rclcpp_action::CancelResponse::ACCEPT;
        }
        
        // Remove from queue if not yet executing
        // Note: priority_queue doesn't support removal, so we'd need to rebuild
        RCLCPP_INFO(this->get_logger(), "   Goal removed from queue (if present)");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * =========================================================================
     * METHOD: handle_accepted
     * =========================================================================
     * Adds accepted goals to the processing queue
     */
    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        
        // Determine priority (could be based on student ID, assignment, etc.)
        GoalPriority priority = GoalPriority::NORMAL;
        if (goal->processing_time_seconds <= 2.0) {
            priority = GoalPriority::HIGH;
        } else if (goal->processing_time_seconds >= 10.0) {
            priority = GoalPriority::LOW;
        }
        
        // Add to queue
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            goal_queue_.emplace(goal, goal_handle, priority);
            RCLCPP_INFO(this->get_logger(), "📥 Goal queued (Priority: %d, Queue size: %zu)", 
                       static_cast<int>(priority), goal_queue_.size());
        }
        
        // Notify worker thread
        queue_cv_.notify_one();
    }

    /**
     * =========================================================================
     * METHOD: process_goal_queue
     * =========================================================================
     * Worker thread that processes goals from the queue
     */
    void process_goal_queue()
    {
        while (!shutdown_requested_) {
            QueuedGoal next_goal(nullptr, nullptr);
            
            // Wait for goals in queue
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait(lock, [this]() { 
                    return shutdown_requested_ || !goal_queue_.empty(); 
                });
                
                if (shutdown_requested_) break;
                
                if (!goal_queue_.empty()) {
                    next_goal = goal_queue_.top();
                    goal_queue_.pop();
                }
            }
            
            if (next_goal.goal) {
                execute_goal(next_goal);
            }
        }
    }

    /**
     * =========================================================================
     * METHOD: execute_goal
     * =========================================================================
     * Executes a single goal with preemption checking
     */
    void execute_goal(QueuedGoal& queued_goal)
    {
        auto goal = queued_goal.goal;
        auto goal_handle = queued_goal.goal_handle;
        
        // Set as currently executing
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            current_goal_handle_ = goal_handle;
            is_processing_ = true;
        }
        
        RCLCPP_INFO(this->get_logger(), "🔨 Executing goal for %s (Priority: %d)", 
                   goal->student_id.c_str(), static_cast<int>(queued_goal.priority));
        
        auto result = std::make_shared<ProcessStudentSubmission::Result>();
        auto feedback = std::make_shared<ProcessStudentSubmission::Feedback>();
        
        const auto processing_time = goal->processing_time_seconds;
        const int total_steps = 10;
        const auto step_duration = processing_time / total_steps;
        
        std::vector<std::string> processing_stages = {
            "Uploading submission...", "Syntax analysis...", "Code quality check...",
            "Plagiarism detection...", "Test case execution...", "Performance benchmarking...",
            "Style guide verification...", "Documentation review...", "Final grading...",
            "Generating feedback..."
        };
        
        try {
            for (int step = 0; step < total_steps && rclcpp::ok(); step++) {
                // Check for preemption (new high-priority goal)
                if (should_preempt_current_goal()) {
                    RCLCPP_INFO(this->get_logger(), "🔄 Preempting current goal for higher priority goal");
                    result->success = false;
                    result->completion_message = "Goal preempted by higher priority goal";
                    goal_handle->abort(result);
                    break;
                }
                
                // Check for cancellation
                if (goal_handle->is_canceling()) {
                    result->success = false;
                    result->completion_message = "Goal cancelled by user";
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "❌ Goal cancelled");
                    break;
                }
                
                // Update and publish feedback
                feedback->progress_percentage = (step * 100.0) / total_steps;
                feedback->current_stage = processing_stages[step];
                feedback->status_message = "Step " + std::to_string(step + 1) + "/" + std::to_string(total_steps);
                
                // Simulate plagiarism detection
                if (step == 3) {
                    feedback->is_plagiarism_detected = (std::rand() % 100 < 20);
                }
                
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "📊 Progress: %.1f%% - %s", 
                           feedback->progress_percentage, feedback->current_stage.c_str());
                
                // Simulate processing
                std::this_thread::sleep_for(std::chrono::duration<float>(step_duration));
                
                // If this is the last step, mark as succeeded
                if (step == total_steps - 1) {
                    std::hash<std::string> hasher;
                    float base_grade = (hasher(goal->student_id) % 71) + 30.0f;
                    
                    result->final_grade = base_grade;
                    result->detailed_feedback = "Submission processed with advanced queuing system";
                    result->plagiarism_status = feedback->is_plagiarism_detected ? "SUSPECTED" : "CLEAN";
                    result->success = true;
                    result->completion_message = "Processing completed successfully";
                    
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "✅ Goal completed for %s", goal->student_id.c_str());
                }
            }
        } catch (const std::exception& ex) {
            result->success = false;
            result->completion_message = "Error: " + std::string(ex.what());
            goal_handle->abort(result);
            RCLCPP_ERROR(this->get_logger(), "💥 Goal execution failed: %s", ex.what());
        }
        
        // Clear current goal
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            current_goal_handle_.reset();
            is_processing_ = false;
        }
    }

    /**
     * =========================================================================
     * METHOD: should_preempt_current_goal
     * =========================================================================
     * Checks if current goal should be preempted for a higher priority one
     */
    bool should_preempt_current_goal()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (goal_queue_.empty()) return false;
        
        // Get the highest priority goal in queue
        auto highest_queued_priority = static_cast<int>(goal_queue_.top().priority);
        
        // Preempt if there's a higher priority goal waiting
        // In real system, you might have more complex logic here
        return highest_queued_priority > static_cast<int>(GoalPriority::NORMAL);
    }

    /**
     * =========================================================================
     * METHOD: publish_status
     * =========================================================================
     * Periodically publishes server status
     */
    void publish_status()
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        RCLCPP_INFO(this->get_logger(), "📊 SERVER STATUS - Processing: %s, Queue: %zu goals", 
                   is_processing_ ? "YES" : "NO", goal_queue_.size());
        
        if (is_processing_ && current_goal_handle_) {
            auto goal = current_goal_handle_->get_goal();
            RCLCPP_INFO(this->get_logger(), "   Current: %s - %s", 
                       goal->student_id.c_str(), goal->assignment_name.c_str());
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
    
    // Seed random number generator
    std::srand(std::time(nullptr));
    
    auto server = std::make_shared<AdvancedSubmissionServer>();
    
    RCLCPP_INFO(server->get_logger(), "🎯 Advanced Goal Management Server Ready!");
    RCLCPP_INFO(server->get_logger(), "💡 Features demonstrated:");
    RCLCPP_INFO(server->get_logger(), "   - Goal Preemption");
    RCLCPP_INFO(server->get_logger(), "   - Priority-based Queuing");
    RCLCPP_INFO(server->get_logger(), "   - Concurrent Goal Management");
    
    rclcpp::spin(server);
    rclcpp::shutdown();
    
    return 0;
}
