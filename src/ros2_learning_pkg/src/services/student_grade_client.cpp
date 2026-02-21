/**
 * Student Grade Service Client
 * 
 * This node creates a service client that sends requests to the student grade service.
 * It demonstrates:
 * - Creating service clients
 * - Building and sending service requests
 * - Handling service responses asynchronously
 * - Error handling for service calls
 * 
 * Usage:
 *   ros2 run ros2_learning_pkg student_grade_client <student_id> <assignment_name>
 * Example:
 *   ros2 run ros2_learning_pkg student_grade_client "S12345" "ROS2_Assignment_1"
 */

#include "rclcpp/rclcpp.hpp"
// Include the generated header for our custom service
#include "ros2_learning_pkg/srv/student_grade.hpp"

// Use shorter namespace for our service type
using StudentGrade = ros2_learning_pkg::srv::StudentGrade;

/**
 * @class StudentGradeClient
 * @brief ROS 2 Node that acts as a client for the student grading service
 * 
 * This node sends requests to the grading service and displays the results.
 * It can be used with command-line arguments or with default values.
 */
class StudentGradeClient : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the StudentGradeClient node
   * @param student_id Default student ID to use
   * @param assignment_name Default assignment name to use
   */
  StudentGradeClient(const std::string& student_id, const std::string& assignment_name) 
    : Node("student_grade_client"),
      student_id_(student_id),
      assignment_name_(assignment_name)
  {
    // Create a client for the StudentGrade service
    // The service name must match what the server is using ("get_student_grade")
    client_ = this->create_client<StudentGrade>("get_student_grade");
    
    RCLCPP_INFO(this->get_logger(), 
                "🎓 Student Grade Service Client started");
    RCLCPP_INFO(this->get_logger(), 
                "Target service: /get_student_grade");
    
    // Wait for the service to be available
    this->wait_for_service();
  }

  /**
   * @brief Send a request to the grading service
   * 
   * This function creates a request with the provided student ID and assignment name,
   * then sends it to the service and waits for the response.
   */
  void send_request()
  {
    RCLCPP_INFO(this->get_logger(), 
                "📤 Sending grade request for Student: %s, Assignment: %s",
                student_id_.c_str(), assignment_name_.c_str());

    // Create the request message
    auto request = std::make_shared<StudentGrade::Request>();
    request->student_id = student_id_;
    request->assignment_name = assignment_name_;

    // Send the request asynchronously and wait for the response
    // We use std::chrono::seconds(5) to set a 5-second timeout
    auto future = client_->async_send_request(request);
    
    // Wait for the response with a timeout
    auto wait_result = rclcpp::spin_until_future_complete(
      this->shared_from_this(), 
      future, 
      std::chrono::seconds(5));
    
    // Check if we got a response within the timeout
    if (wait_result == rclcpp::FutureReturnCode::SUCCESS) {
      // Successfully received a response
      auto response = future.get();
      
      if (response->success) {
        // Request was successful - display the grade and feedback
        RCLCPP_INFO(this->get_logger(), "✅ Grade Request Successful!");
        RCLCPP_INFO(this->get_logger(), "   Student ID: %s", student_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "   Assignment: %s", assignment_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "   Grade: %.1f/100.0", response->grade);
        RCLCPP_INFO(this->get_logger(), "   Status: %s", response->message.c_str());
        RCLCPP_INFO(this->get_logger(), "   Feedback: %s", response->feedback.c_str());
      } else {
        // Service processed the request but indicated failure
        RCLCPP_ERROR(this->get_logger(), "❌ Service returned error: %s", 
                     response->message.c_str());
      }
    } else {
      // Timeout or other error occurred
      RCLCPP_ERROR(this->get_logger(), 
                   "💥 Failed to get response from service within timeout");
      RCLCPP_INFO(this->get_logger(), 
                  "   Make sure the service server is running: ros2 run ros2_learning_pkg student_grade_server");
    }
  }

private:
  // Client handle - used to send requests to the service
  rclcpp::Client<StudentGrade>::SharedPtr client_;
  std::string student_id_;
  std::string assignment_name_;

  /**
   * @brief Wait for the service to become available
   * 
   * This function checks if the service server is running and available.
   * It will wait up to 2 seconds for the service to appear.
   */
  void wait_for_service()
  {
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for service to be available...");
    
    // Wait for the service to be available (max 2 seconds)
    if (!client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(), 
                   "💥 Service not available after waiting!");
      RCLCPP_INFO(this->get_logger(), 
                  "   Please start the server first: ros2 run ros2_learning_pkg student_grade_server");
      rclcpp::shutdown();
      exit(1);
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ Service is available!");
  }
};

/**
 * @brief Main function that parses command line arguments and runs the client
 */
int main(int argc, char **argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Default values for student ID and assignment name
  std::string student_id = "S001";
  std::string assignment_name = "ROS2_Midterm";

  // Parse command line arguments if provided
  // argc[0] is program name, argc[1] is first argument, etc.
  if (argc >= 3) {
    // User provided both student ID and assignment name
    student_id = argv[1];
    assignment_name = argv[2];
    RCLCPP_INFO(rclcpp::get_logger("main"), 
                "Using provided values - Student: %s, Assignment: %s", 
                student_id.c_str(), assignment_name.c_str());
  } else if (argc == 2) {
    // User provided only student ID
    student_id = argv[1];
    RCLCPP_WARN(rclcpp::get_logger("main"), 
                "Only student ID provided, using default assignment name");
  } else {
    // No arguments provided, using defaults
    RCLCPP_INFO(rclcpp::get_logger("main"), 
                "No arguments provided, using default values");
    RCLCPP_INFO(rclcpp::get_logger("main"), 
                "Usage: ros2 run ros2_learning_pkg student_grade_client <student_id> <assignment_name>");
  }

  // Create and run the client
  auto client_node = std::make_shared<StudentGradeClient>(student_id, assignment_name);
  client_node->send_request();

  // Clean shutdown
  rclcpp::shutdown();
  return 0;
}
