/**
 * =============================================================================
 * Advanced Student Grade Service Server
 * =============================================================================
 * 
 * DEMONSTRATES: Service timeout scenarios and delayed responses
 * 
 * FEATURES:
 * - Simulated processing delays
 * - Random success/failure for testing
 * - Multiple response time patterns
 * - Graceful error handling
 * 
 * USAGE:
 *   ros2 run ros2_learning_pkg advanced_grade_server [mode]
 * 
 * MODES:
 *   fast = Immediate responses (default)
 *   slow = 3-second delays  
 *   random = Random delays and failures
 *   timeout = Always timeout (for testing client timeouts)
 */

#include "rclcpp/rclcpp.hpp"
#include "ros2_learning_pkg/srv/student_grade.hpp"
#include <random>
#include <chrono>

using namespace std::chrono_literals;
using StudentGrade = ros2_learning_pkg::srv::StudentGrade;

class AdvancedGradeServer : public rclcpp::Node
{
public:
    AdvancedGradeServer(const std::string& mode = "fast") 
        : Node("advanced_grade_server"), mode_(mode)
    {
        service_ = this->create_service<StudentGrade>(
            "get_student_grade",
            std::bind(&AdvancedGradeServer::handle_grade_request, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Initialize random number generator for random mode
        rng_.seed(std::random_device{}());
        
        RCLCPP_INFO(this->get_logger(), "🚀 Advanced Grade Server Started");
        RCLCPP_INFO(this->get_logger(), "📡 Service: /get_student_grade");
        RCLCPP_INFO(this->get_logger(), "🎛️  Mode: %s", mode_.c_str());
        RCLCPP_INFO(this->get_logger(), "⏳ Ready for requests...");
    }

private:
    rclcpp::Service<StudentGrade>::SharedPtr service_;
    std::string mode_;
    std::mt19937 rng_; // Random number generator

    void handle_grade_request(
        const std::shared_ptr<StudentGrade::Request> request,
        const std::shared_ptr<StudentGrade::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), 
                   "📥 Received request - Student: %s, Assignment: %s",
                   request->student_id.c_str(), request->assignment_name.c_str());

        // SIMULATE DIFFERENT RESPONSE PATTERNS BASED ON MODE
        if (mode_ == "slow") {
            RCLCPP_WARN(this->get_logger(), "   ⏰ SLOW MODE: Simulating 3-second processing...");
            std::this_thread::sleep_for(3s);
        }
        else if (mode_ == "timeout") {
            RCLCPP_ERROR(this->get_logger(), "   💥 TIMEOUT MODE: Simulating server freeze...");
            std::this_thread::sleep_for(10s); // Longer than client timeout
            RCLCPP_INFO(this->get_logger(), "   ⏰ Finally responding (too late for client)");
        }
        else if (mode_ == "random") {
            std::uniform_int_distribution<int> dist(1, 100);
            int random_value = dist(rng_);
            
            if (random_value <= 20) { // 20% chance of failure
                RCLCPP_ERROR(this->get_logger(), "   🎲 RANDOM MODE: Simulating server error");
                response->success = false;
                response->grade = 0.0;
                response->feedback = "Server error: Internal processing failure";
                response->message = "SERVER_ERROR";
                return;
            }
            
            // Random delay between 1-4 seconds
            int delay_seconds = (random_value % 4) + 1;
            RCLCPP_WARN(this->get_logger(), "   🎲 RANDOM MODE: %d second delay", delay_seconds);
            std::this_thread::sleep_for(std::chrono::seconds(delay_seconds));
        }

        // NORMAL PROCESSING (after potential delays)
        if (request->student_id.empty() || request->assignment_name.empty()) {
            response->success = false;
            response->grade = 0.0;
            response->feedback = "Error: Invalid request parameters";
            response->message = "INVALID_REQUEST";
            RCLCPP_WARN(this->get_logger(), "   ❌ Invalid request processed");
            return;
        }

        // Calculate grade (same logic as basic server)
        std::hash<std::string> hasher;
        float base_grade = (hasher(request->student_id) % 71) + 30.0f;
        float adjustment = (hasher(request->assignment_name) % 21) - 10.0f;
        response->grade = std::min(100.0f, std::max(0.0f, base_grade + adjustment));
        response->success = true;

        // Generate feedback
        if (response->grade >= 90.0) {
            response->feedback = "Excellent work in " + mode_ + " mode!";
            response->message = "EXCELLENT";
        } else if (response->grade >= 80.0) {
            response->feedback = "Good job in " + mode_ + " mode!";
            response->message = "GOOD";
        } else if (response->grade >= 70.0) {
            response->feedback = "Satisfactory in " + mode_ + " mode";
            response->message = "SATISFACTORY";
        } else if (response->grade >= 60.0) {
            response->feedback = "Needs improvement in " + mode_ + " mode";
            response->message = "NEEDS_IMPROVEMENT";
        } else {
            response->feedback = "Unsatisfactory in " + mode_ + " mode";
            response->message = "FAILING";
        }

        RCLCPP_INFO(this->get_logger(), 
                   "📤 Response sent - Grade: %.1f, Status: %s",
                   response->grade, response->message.c_str());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    std::string mode = "fast";
    if (argc >= 2) {
        mode = argv[1];
    }
    
    auto server = std::make_shared<AdvancedGradeServer>(mode);
    rclcpp::spin(server);
    rclcpp::shutdown();
    
    return 0;
}
