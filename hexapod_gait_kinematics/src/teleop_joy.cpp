#include "hexapod_gait_kinematics/teleop_joy.hpp"
#include <chrono>

TeleopJoy::TeleopJoy() : Node("teleop_joy") {
    this->declare_parameter<double>("clearance", 0.045);
    this->get_parameter("clearance", z);

    body_state.leg_radius = 0.5;
    body_state.z = -z;
    start_flag = false;

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy_out", 1,
        std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1),
        sub_options
    );

    move_body_pub = this->create_publisher<hexapod_msgs::msg::BodyState>("/teleop/move_body", 1);
    body_cmd_pub = this->create_publisher<hexapod_msgs::msg::BodyCommand>("/teleop/body_command", 1);
    gait_cmd_pub = this->create_publisher<hexapod_msgs::msg::GaitCommand>("/teleop/gait_control", 1);
}

void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy) {
    if (joy->buttons[button_start] && !imu_flag) {
        if (!start_flag) {
            start_flag = true;
            body_command.cmd = body_command.STAND_UP_CMD;
            body_cmd_pub->publish(body_command);
        } else {
            start_flag = false;
            body_command.cmd = body_command.SEAT_DOWN_CMD;
            body_cmd_pub->publish(body_command);
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (joy->buttons[button_imu] && !start_flag) {
        if (!imu_flag) {
            imu_flag = true;
            body_command.cmd = body_command.IMU_START_CMD;
            body_cmd_pub->publish(body_command);
        } else {
            imu_flag = false;
            body_command.cmd = body_command.IMU_STOP_CMD;
            body_cmd_pub->publish(body_command);
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    if (joy->buttons[button_gait_switch]) {
        if (!gait_flag) {
            gait_flag = true;
            gait_command.cmd = gait_command.STOP;
        } else {
            gait_flag = false;
            gait_command.cmd = gait_command.STOP;
        }
        gait_cmd_pub->publish(gait_command);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    if (start_flag) {
        if (joy->buttons[button_left_shift]) {
            body_state.roll = 0.25 * joy->axes[axis_body_roll];
            body_state.pitch = -0.25 * joy->axes[axis_body_pitch];
            body_state.yaw = -0.28 * joy->axes[axis_body_yaw];
            move_body_pub->publish(body_state);
        }
        if (joy->buttons[button_right_shift]) {
            body_state.y = -0.05 * joy->axes[axis_body_y_off];
            body_state.x = -0.05 * joy->axes[axis_body_x_off];
            if (joy->axes[axis_body_z_off] < 0) {
                body_state.z = -0.03 * joy->axes[axis_body_z_off] - z;
            } else {
                body_state.z = -0.1 * joy->axes[axis_body_z_off] - z;
            }
            move_body_pub->publish(body_state);
        }
        if (!joy->buttons[button_left_shift] && !joy->buttons[button_right_shift]) {
            if (joy->axes[axis_fi_x] != 0 || joy->axes[axis_fi_y] != 0) {
                if (!gait_flag) {
                    gait_command.cmd = gait_command.RUNRIPPLE;
                } else {
                    gait_command.cmd = gait_command.RUNTRIPOD;
                }
                float a, b;
                a = pow(joy->axes[axis_fi_x], 2);
                b = pow(joy->axes[axis_fi_y], 2);
                gait_command.fi = atan2(joy->axes[axis_fi_x], joy->axes[axis_fi_y]);
                gait_command.scale = pow(a + b, 0.5) > 1 ? 1 : pow(a + b, 0.5);
                gait_command.alpha = 0;
            }

            if (joy->axes[axis_alpha] != 0 || joy->axes[axis_scale] != 0) {
                if (!gait_flag) {
                    gait_command.cmd = gait_command.RUNRIPPLE;
                } else {
                    gait_command.cmd = gait_command.RUNTRIPOD;
                }
                gait_command.fi = (joy->axes[axis_scale] > 0) ? 0 : 3.14;
                gait_command.scale = joy->axes[axis_scale];
                if (gait_command.scale < 0) gait_command.scale *= -1;
                gait_command.alpha = ((joy->axes[axis_alpha] > 0) ? 1 : -1) * 0.06 * (1 - gait_command.scale) + 0.11 * joy->axes[axis_alpha];
            }

            if (!joy->axes[axis_alpha] && !joy->axes[axis_scale] && !joy->axes[axis_fi_x] && !joy->axes[axis_fi_y]) {
                gait_command.cmd = gait_command.PAUSE;
            }
            gait_cmd_pub->publish(gait_command);
        }
    } else {
        if (joy->buttons[button_right_shift_2] && !imu_flag) {
            body_state.z = -0.01;
            body_state.leg_radius = 0.06 * joy->axes[axis_body_yaw] + 0.5;
            move_body_pub->publish(body_state);
        }
        gait_command.cmd = gait_command.STOP;
        gait_cmd_pub->publish(gait_command);
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting ps3 teleop converter, take care of your controller now...");
    auto node = std::make_shared<TeleopJoy>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
