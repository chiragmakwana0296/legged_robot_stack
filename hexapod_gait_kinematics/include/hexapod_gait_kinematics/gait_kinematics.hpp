

#ifndef HEXAPOD_GAIT_KINEMATICS__GAIT_KINEMATICS_HPP_
#define HEXAPOD_GAIT_KINEMATICS__GAIT_KINEMATICS_HPP_

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
//#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <hexapod_msgs/GetIKSolver.h>
#include <hexapod_msgs/LegsJointsState.h>
#include <hexapod_msgs/GaitCommand.h>
#include "hexapod_gait_kinematics/gait.hpp"

#define NUM_LEGS 6
#define NUM_JOINTS 6

class GaitKinematics {
	public:
		GaitKinematics();
		bool init();
		void gaitGenerator();

	private:
		ros::NodeHandle node;
		std::string root_name, tip_name;
		std::vector<KDL::Frame> frames;
		double fi;
		hexapod_msgs::LegsJointsState legs;
		hexapod_msgs::GetIKSolver srv;
		hexapod_msgs::GaitCommand gait_command;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;
		double trap_low_r, trap_high_r, trap_h, trap_z;
		double d_ripple, d_tripod;

		ros::ServiceClient client;
		ros::Publisher joints_pub;
		ros::Subscriber gait_control_sub;

		bool loadModel(const std::string xml);
		bool callService (KDL::Vector* vector);
		void teleopGaitCtrl (const hexapod_msgs::GaitCommandConstPtr &gait_cmd);

};

GaitKinematics::GaitKinematics(){}



#endif /* HEXAPOD_BODYKINEMATICS__GAIT_KINEMATICS_HPP_ */
