#ifndef TELEOP_TWIST_HPP_
#define TELEOP_TWIST_HPP_

#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hexapod_msgs/msg/gait_command.hpp"
#include "hexapod_msgs/msg/body_command.hpp"
#include "std_msgs/msg/bool.hpp"

class TeleopTwist : public rclcpp::Node {
public:
    TeleopTwist();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void standCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void timerCallback();

    // Parameters
    double max_linear_vel_;
    double max_angular_vel_;
    double max_alpha_;
    int gait_type_;
    double watchdog_timeout_;
    bool auto_stand_up_;

    // Publishers & Subscribers
    rclcpp::Publisher<hexapod_msgs::msg::GaitCommand>::SharedPtr gait_pub_;
    rclcpp::Publisher<hexapod_msgs::msg::BodyCommand>::SharedPtr body_cmd_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stand_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    geometry_msgs::msg::Twist last_twist_;
    rclcpp::Time last_cmd_time_;
    bool has_received_cmd_;
    bool is_stood_up_;
};

#endif // TELEOP_TWIST_HPP_
