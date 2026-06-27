#include "hexapod_teleop/teleop_twist.hpp"
#include <cmath>

using namespace std::chrono_literals;

TeleopTwist::TeleopTwist() : Node("teleop_twist") {
    // Declare parameters
    this->declare_parameter<double>("max_linear_velocity", 0.2);
    this->declare_parameter<double>("max_angular_velocity", 0.5);
    this->declare_parameter<double>("max_alpha", 0.15);
    this->declare_parameter<int>("gait_type", 2); // 2 = TRIPOD, 1 = RIPPLE
    this->declare_parameter<double>("watchdog_timeout", 0.5); // seconds
    this->declare_parameter<bool>("auto_stand_up", true);

    // Get parameters
    this->get_parameter("max_linear_velocity", max_linear_vel_);
    this->get_parameter("max_angular_velocity", max_angular_vel_);
    this->get_parameter("max_alpha", max_alpha_);
    this->get_parameter("gait_type", gait_type_);
    this->get_parameter("watchdog_timeout", watchdog_timeout_);
    this->get_parameter("auto_stand_up", auto_stand_up_);

    // Publishers
    gait_pub_ = this->create_publisher<hexapod_msgs::msg::GaitCommand>("/teleop/gait_control", 10);
    body_cmd_pub_ = this->create_publisher<hexapod_msgs::msg::BodyCommand>("/teleop/body_command", 10);

    // Subscribers
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&TeleopTwist::cmdVelCallback, this, std::placeholders::_1)
    );

    stand_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/teleop/stand", 10,
        std::bind(&TeleopTwist::standCallback, this, std::placeholders::_1)
    );

    // Timer for watchdog and publishing commands
    timer_ = this->create_wall_timer(
        50ms, // 20Hz
        std::bind(&TeleopTwist::timerCallback, this)
    );

    last_cmd_time_ = this->now();
    has_received_cmd_ = false;
    is_stood_up_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Teleop Twist Node initialized. Listening on /cmd_vel");
}

void TeleopTwist::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_twist_ = *msg;
    last_cmd_time_ = this->now();
    has_received_cmd_ = true;
}

void TeleopTwist::standCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    hexapod_msgs::msg::BodyCommand cmd;
    if (msg->data) {
        cmd.cmd = hexapod_msgs::msg::BodyCommand::STAND_UP_CMD;
        is_stood_up_ = true;
        RCLCPP_INFO(this->get_logger(), "Sending Stand Up command");
    } else {
        cmd.cmd = hexapod_msgs::msg::BodyCommand::SEAT_DOWN_CMD;
        is_stood_up_ = false;
        RCLCPP_INFO(this->get_logger(), "Sending Sit Down command");
    }
    body_cmd_pub_->publish(cmd);
}

void TeleopTwist::timerCallback() {
    if (!has_received_cmd_) {
        return;
    }

    double dt = (this->now() - last_cmd_time_).seconds();
    if (dt > watchdog_timeout_) {
        // Stop/pause robot due to timeout
        hexapod_msgs::msg::GaitCommand stop_cmd;
        stop_cmd.cmd = hexapod_msgs::msg::GaitCommand::PAUSE;
        gait_pub_->publish(stop_cmd);
        return;
    }

    double vx = last_twist_.linear.x;
    double vy = last_twist_.linear.y;
    double wz = last_twist_.angular.z;

    if (std::abs(vx) < 1e-4 && std::abs(vy) < 1e-4 && std::abs(wz) < 1e-4) {
        hexapod_msgs::msg::GaitCommand stop_cmd;
        stop_cmd.cmd = hexapod_msgs::msg::GaitCommand::PAUSE;
        gait_pub_->publish(stop_cmd);
        return;
    }

    // Auto stand up if configured and not already stood up
    if (auto_stand_up_ && !is_stood_up_) {
        hexapod_msgs::msg::BodyCommand stand_cmd;
        stand_cmd.cmd = hexapod_msgs::msg::BodyCommand::STAND_UP_CMD;
        body_cmd_pub_->publish(stand_cmd);
        is_stood_up_ = true;
        RCLCPP_INFO(this->get_logger(), "Auto standing up before walking");
        // Wait a brief moment to allow stand up to start/finish
        rclcpp::sleep_for(500ms);
    }

    hexapod_msgs::msg::GaitCommand gait_cmd;
    gait_cmd.cmd = gait_type_;

    // Calculate direction of movement (fi)
    // fi is the angle in the XY plane.
    // x is forward, y is left.
    gait_cmd.fi = std::atan2(vy, vx);

    // Calculate velocity magnitude and scale it to [0, 1]
    double linear_speed = std::hypot(vx, vy);
    gait_cmd.scale = linear_speed / max_linear_vel_;
    if (gait_cmd.scale > 1.0) {
        gait_cmd.scale = 1.0;
    }

    // Calculate alpha (yaw rotation of the gait)
    gait_cmd.alpha = (wz / max_angular_vel_) * max_alpha_;
    if (gait_cmd.alpha > max_alpha_) {
        gait_cmd.alpha = max_alpha_;
    } else if (gait_cmd.alpha < -max_alpha_) {
        gait_cmd.alpha = -max_alpha_;
    }

    // If linear speed is zero but angular velocity is non-zero, we rotate in place.
    // In this case, we set fi to 0 (or pi depending on direction, though it doesn't matter much since scale is 0)
    // and set scale to 0, but set alpha to non-zero.
    if (linear_speed < 1e-4) {
        gait_cmd.scale = 0.0;
        gait_cmd.fi = 0.0;
    }

    gait_pub_->publish(gait_cmd);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTwist>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
