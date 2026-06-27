#ifndef HEXAPOD_GAIT_KINEMATICS__GAIT_KINEMATICS_HPP_
#define HEXAPOD_GAIT_KINEMATICS__GAIT_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <hexapod_msgs/srv/get_ik_solver.hpp>
#include <hexapod_msgs/msg/legs_joints_state.hpp>
#include <hexapod_msgs/msg/gait_command.hpp>
#include "hexapod_gait_kinematics/gait.hpp"
#include <geometry_msgs/msg/pose_array.hpp>

#define NUM_LEGS 6
#define NUM_JOINTS 6

class GaitKinematics : public rclcpp::Node {
	public:
		GaitKinematics();
		bool init();

	private:
		std::string root_name, tip_name;
		std::vector<KDL::Frame> frames;
		double fi;
		hexapod_msgs::msg::LegsJointsState legs;
		hexapod_msgs::msg::GaitCommand gait_command;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;
		double trap_low_r, trap_high_r, trap_h, trap_z;
		double d_ripple, d_tripod;
		Gait gait;

		rclcpp::CallbackGroup::SharedPtr cb_group_;
		rclcpp::Client<hexapod_msgs::srv::GetIKSolver>::SharedPtr client;
		rclcpp::Publisher<hexapod_msgs::msg::LegsJointsState>::SharedPtr joints_pub;
		rclcpp::Subscription<hexapod_msgs::msg::GaitCommand>::SharedPtr gait_control_sub;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr leg_target_pose_viz_pub;
		rclcpp::TimerBase::SharedPtr timer_;
		
		void timerCallback();
		bool loadModel(const std::string xml);
		bool callService (KDL::Vector* vector);
		void teleopGaitCtrl (const hexapod_msgs::msg::GaitCommand::SharedPtr gait_cmd);
};

#endif /* HEXAPOD_GAIT_KINEMATICS__GAIT_KINEMATICS_HPP_ */
