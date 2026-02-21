/**
 * Student Grade Service Server
 * 
 * This node creates a service server that responds to student grade requests.
 * It demonstrates:
 * - Custom service implementation
 * - Service request handling
 * - Response generation with custom data
 * 
 * Usage:
 *   ros2 run ros2_learning_pkg student_grade_server
 */

#include "rclcpp/rclcpp.hpp"
// Include the generated header for our custom service
// The header is automatically generated from StudentGrade.srv
#include "ros2_learning_pkg/srv/student_grade.hpp"

// Use shorter namespace for our service type to make code cleaner
using StudentGrade = ros2_learning_pkg::srv::StudentGrade;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @class StudentGradeServer
 * @brief ROS 2 Node that provides a student grading service
 * 
 * This node creates a service that simulates a grading system.
 * It receives student IDs and assignment names, then returns
 * simulated grades and feedback.
 */
class StudentGradeServer : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the StudentGradeServer node
   */
  StudentGradeServer() : Node("student_grade_server")
  {
    // Create a service named 'get_student_grade' that uses our custom service type
    // std::bind connects the service callback to this class instance
    service_ = this->create_service<StudentGrade>(
      "get_student_grade",
      std::bind(&StudentGradeServer::handle_grade_request, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), 
                "🎓 Student Grade Service Server started and waiting for requests...");
    RCLCPP_INFO(this->get_logger(), 
                "Service name: /get_student_grade");
  }

private:
  // Service handle - manages the service and incoming requests
  rclcpp::Service<StudentGrade>::SharedPtr service_;

  /**
   * @brief Callback function that handles incoming grade requests
   * 
   * This function is automatically called when a client sends a request
   * to the 'get_student_grade' service.
   * 
   * @param request The incoming request containing student_id and assignment_name
   * @param response The outgoing response that we'll fill with grade data
   */
  void handle_grade_request(
    const std::shared_ptr<StudentGrade::Request> request,
    const std::shared_ptr<StudentGrade::Response> response)
  {
    // Log the incoming request for debugging
    RCLCPP_INFO(this->get_logger(), 
                "📥 Received grade request - Student: %s, Assignment: %s", 
                request->student_id.c_str(), 
                request->assignment_name.c_str());

    // Simulate a grading system - in real life, this would query a database
    // For demonstration, we'll generate grades based on the input
    
    // Check if we have data for this student
    if (request->student_id.empty() || request->assignment_name.empty()) {
      response->success = false;
      response->grade = 0.0;
      response->feedback = "Error: Student ID and Assignment name cannot be empty";
      response->message = "INVALID_REQUEST";
      
      RCLCPP_WARN(this->get_logger(), "Invalid request received - empty fields");
      return;
    }

    // Simulate different grades based on student ID and assignment
    // This is just for demonstration - real system would have actual logic
    
    // Generate a "deterministic random" grade based on string hashes
    // This ensures the same student+assignment always gets the same grade
    size_t student_hash = std::hash<std::string>{}(request->student_id);
    size_t assignment_hash = std::hash<std::string>{}(request->assignment_name);
    float base_grade = (student_hash % 71) + 30; // Grade between 30-100
    float adjustment = (assignment_hash % 21) - 10; // Adjustment between -10 to +10
    
    response->grade = std::min(100.0f, std::max(0.0f, base_grade + adjustment));
    response->success = true;
    
    // Generate appropriate feedback based on the grade
    if (response->grade >= 90.0) {
      response->feedback = "Excellent work! You demonstrated mastery of the concepts.";
      response->message = "EXCELLENT";
    } else if (response->grade >= 80.0) {
      response->feedback = "Good job! Solid understanding with minor areas for improvement.";
      response->message = "GOOD";
    } else if (response->grade >= 70.0) {
      response->feedback = "Satisfactory. Review the material and practice more.";
      response->message = "SATISFACTORY";
    } else if (response->grade >= 60.0) {
      response->feedback = "Needs improvement. Please review the fundamentals.";
      response->message = "NEEDS_IMPROVEMENT";
    } else {
      response->feedback = "Unsatisfactory. Significant study required.";
      response->message = "FAILING";
    }

    // Add assignment-specific feedback
    response->feedback += " Assignment: " + request->assignment_name + " completed.";

    // Log the response for monitoring
    RCLCPP_INFO(this->get_logger(),
                "📤 Sending response - Grade: %.1f, Status: %s",
                response->grade, response->message.c_str());
  }
};

/**
 * @brief Main function that initializes ROS 2 and runs the service server
 */
int main(int argc, char **argv)
{
  // Initialize ROS 2 with the node name
  rclcpp::init(argc, argv);
  
  // Create the service server node and keep it running
  auto node = std::make_shared<StudentGradeServer>();
  
  // Spin keeps the node alive and listening for service requests
  RCLCPP_INFO(node->get_logger(), "🚀 Starting Student Grade Service Server...");
  rclcpp::spin(node);
  
  // Clean shutdown when the node is stopped
  rclcpp::shutdown();
  return 0;
}
