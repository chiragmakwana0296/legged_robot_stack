#ifndef HEXAPOD_BODY_KINEMATICS__BODY_KINEMATICS_HPP_
#define HEXAPOD_BODY_KINEMATICS__BODY_KINEMATICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <hexapod_msgs/srv/get_ik_solver.hpp>
#include <hexapod_msgs/msg/legs_joints_state.hpp>
#include <hexapod_msgs/msg/body_state.hpp>
#include <hexapod_msgs/msg/body_command.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#define NUM_LEGS 6
#define NUM_JOINTS 3

class BodyKinematics : public rclcpp::Node {
	public:
		BodyKinematics();
		bool init();

	private:
		std::string root_name, tip_name;
		std::vector<KDL::Frame> frames;
		hexapod_msgs::msg::BodyState bs;
		hexapod_msgs::msg::LegsJointsState legs;
		std::shared_ptr<hexapod_msgs::srv::GetIKSolver::Request> srv_req;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;
		double z;

		KDL::Rotation rotation;
		KDL::Frame tibia_foot_frame, femur_frame;
		KDL::Vector offset_vector, rotate_correction, final_vector [num_legs];

		rclcpp::CallbackGroup::SharedPtr cb_group_;
		rclcpp::TimerBase::SharedPtr startup_timer_;
		rclcpp::Client<hexapod_msgs::srv::GetIKSolver>::SharedPtr client;
		rclcpp::Publisher<hexapod_msgs::msg::LegsJointsState>::SharedPtr joints_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr leg_target_pose_pub;
		rclcpp::Subscription<hexapod_msgs::msg::BodyState>::SharedPtr body_move_sub;
		rclcpp::Subscription<hexapod_msgs::msg::BodyCommand>::SharedPtr body_cmd_sub;

		bool loadModel(const std::string xml);
		bool calculateKinematics (hexapod_msgs::msg::BodyState* body_ptr);
		bool callService (KDL::Vector* vector);
		void teleopBodyMove (const hexapod_msgs::msg::BodyState::SharedPtr body_state);
		void teleopBodyCmd (const hexapod_msgs::msg::BodyCommand::SharedPtr body_cmd);
};

#endif /* HEXAPOD_BODY_KINEMATICS__BODY_KINEMATICS_HPP_ */
