#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
    };
}