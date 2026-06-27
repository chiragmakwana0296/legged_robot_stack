#ifndef TELEOP_JOY_HPP_
#define TELEOP_JOY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <hexapod_msgs/msg/body_state.hpp>
#include <hexapod_msgs/msg/body_command.hpp>
#include <hexapod_msgs/msg/gait_command.hpp>

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

class TeleopJoy : public rclcpp::Node {
	public:
		TeleopJoy();

	private:
		hexapod_msgs::msg::BodyState body_state;
		hexapod_msgs::msg::BodyCommand body_command;
		hexapod_msgs::msg::GaitCommand gait_command;

		rclcpp::CallbackGroup::SharedPtr cb_group_;
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
		rclcpp::Publisher<hexapod_msgs::msg::BodyState>::SharedPtr move_body_pub;
		rclcpp::Publisher<hexapod_msgs::msg::BodyCommand>::SharedPtr body_cmd_pub;
		rclcpp::Publisher<hexapod_msgs::msg::GaitCommand>::SharedPtr gait_cmd_pub;

		double z;

		bool start_flag;
		bool gait_flag;
		bool imu_flag;

		void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

		const static int axis_body_roll = PS3_AXIS_STICK_LEFT_LEFTWARDS;
		const static int axis_body_pitch = PS3_AXIS_STICK_LEFT_UPWARDS;
		const static int axis_body_yaw = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
		const static int axis_body_y_off = PS3_AXIS_STICK_LEFT_LEFTWARDS;
		const static int axis_body_x_off = PS3_AXIS_STICK_LEFT_UPWARDS;
		const static int axis_body_z_off = PS3_AXIS_STICK_RIGHT_UPWARDS;
		const static int button_left_shift = PS3_BUTTON_REAR_LEFT_1;
		const static int button_right_shift = PS3_BUTTON_REAR_RIGHT_1;
		const static int button_right_shift_2 = PS3_BUTTON_REAR_LEFT_2;
		const static int button_start = PS3_BUTTON_START;
		const static int axis_fi_x = PS3_AXIS_STICK_LEFT_LEFTWARDS;
		const static int axis_fi_y = PS3_AXIS_STICK_LEFT_UPWARDS;
		const static int button_gait_switch = PS3_BUTTON_ACTION_TRIANGLE;
		const static int axis_alpha = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
		const static int axis_scale = PS3_AXIS_STICK_RIGHT_UPWARDS;
		const static int button_imu = PS3_BUTTON_ACTION_CROSS;
};

#endif /* TELEOP_JOY_HPP_ */
