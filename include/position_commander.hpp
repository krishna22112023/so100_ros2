#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

namespace motion_position_control
{
    class PositionCommander : public rclcpp::Node
    {
    public:
        PositionCommander();
    private:
        void publish_loop();
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_publisher_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        // Store desired first 6 joint positions loaded from a ROS 2 parameter
        std::vector<double> target_positions_;
    };
}