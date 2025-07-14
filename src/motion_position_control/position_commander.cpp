#include <position_commander.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace motion_position_control;

PositionCommander::PositionCommander():Node("position_commander"){
    // Declare a parameter that allows the user to specify the first 6 desired joint positions
    // Default values keep the previous behaviour (all joints to 1.0)
    this->declare_parameter<std::vector<double>>("target_positions", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    // Retrieve the parameter once at start-up. Users can still change it at runtime
    this->get_parameter("target_positions", target_positions_);
    if (target_positions_.size() != 6) {
        RCLCPP_WARN(this->get_logger(), "Parameter 'target_positions' should contain exactly 6 elements (received %zu). Using defaults instead.", target_positions_.size());
        target_positions_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    }

    position_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>("position_command", 10); //10 refers to publisher queue size
    // the position_commander node pusblishes messages to the "position_command" topic
    // <controller_name>/commands is a standard topic name format
    publish_timer_ = create_wall_timer(std::chrono::milliseconds(1000), 
        std::bind(&PositionCommander::publish_loop, this)); // 1000 ms timer
        //create a callable function object (a "bound function" using std::bind) 
        //that wraps the method PositionCommander::publish_loop and binds it to the current object instance (this).
}

void PositionCommander::publish_loop() {
    std_msgs::msg::Float64MultiArray message;

    // Desired arm joint positions (6 DOF)
    message.data = target_positions_;

    // Default configuration for the cup free joint (7 DOF: xyz + wxyz)
    const std::array<double, 7> cup_default = {0.0, -0.4, 0.05, 1.0, 0.0, 0.0, 0.0};

    // Default configuration for the ball free joint (7 DOF: xyz + wxyz)
    const std::array<double, 7> ball_default = {0.08, -0.2, 0.07, 1.0, 0.0, 0.0, 0.0};

    // Append the defaults to reach the required 20-element vector
    message.data.insert(message.data.end(), cup_default.begin(), cup_default.end());
    message.data.insert(message.data.end(), ball_default.begin(), ball_default.end());

    position_publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Published position command (first 6 joints): [%f, %f, %f, %f, %f, %f]", 
                target_positions_[0], target_positions_[1], target_positions_[2], target_positions_[3], target_positions_[4], target_positions_[5]);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    //argc is the number of command line arguments,
    //argv is an array of C-style strings representing those arguments.
    // Initialize the ROS 2 system and create a node instance and also connects to DDS (Data Distribution Service) middleware.
    rclcpp::spin(std::make_shared<PositionCommander>());
    rclcpp::shutdown();
    return 0;
}

