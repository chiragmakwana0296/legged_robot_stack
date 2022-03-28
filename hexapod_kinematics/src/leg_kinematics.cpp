#include "ros/ros.h"
#include "hexapod_msgs/LegJointsState.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <eigen3/Eigen/Core>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>


void jointStatesCb(const sensor_msgs::JointStateConstPtr &msg){
		// msg->position
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_kinematics_pub");
	ros::NodeHandle node;
	KDL::Tree tree;
	KDL::Chain chain;
	KDL::ChainFkSolverPos_recursive* fk_solver;
	KDL::ChainIkSolverVel_pinv* ik_solver_vel;
	KDL::ChainIkSolverPos_NR* ik_solver_pos;

	ros::Publisher leg_msg_pub = node.advertise<hexapod_msgs::LegJointsState>("leg_states", 1);
	ros::Publisher joint_msg_pub = node.advertise<sensor_msgs::JointState>("/joint_states", 1);
	ros::Publisher target_pose_msg_pub = node.advertise<geometry_msgs::PoseStamped>("/target_pose", 1);
	// ros::Subscriber js_sub = node.subscribe();
	// ros::Subscriber body_cmd_sub = node.subscribe<sensor_msgs::JointState>("/joint_states", 1, &jointStatesCb, this);
	double joint_position[12] = {};
	ros::Rate loop_rate(30);

	std::string robot_desc_string;
	node.param("robot_description", robot_desc_string, std::string("robot_description"));
	if (!kdl_parser::treeFromString(robot_desc_string, tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
		}
	if (!tree.getChain("base_link", "tip_r1", chain)) {
		ROS_ERROR("Failed to construct kdl chain");
		return false;
	   }
	ROS_INFO("Construct kdl chain");

	fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
	ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);
	// Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6,6);
	// matrix_Mx(3,3) = 0; matrix_Mx(4,4) = 0; matrix_Mx(5,5) = 0;
	// ik_solver_vel -> setWeightTS(matrix_Mx);
	KDL::JntArray joint_min(chain.getNrOfJoints());
	KDL::JntArray joint_max(chain.getNrOfJoints());
	joint_min(0) = -3.14159;	joint_min(1) = -3.14159;	joint_min(2) = -3.14159;
	joint_max(0) = 3.14159;		joint_max(1) = 3.14159;		joint_max(2) = 3.14159;
	ik_solver_pos = new KDL::ChainIkSolverPos_NR (chain, *fk_solver, *ik_solver_vel, 1000, 0.01);
	ROS_INFO_STREAM(chain.getNrOfJoints());
	KDL::JntArray q_init(chain.getNrOfJoints());
	q_init (0) = 0.0;	q_init (1) = 0.0;	q_init (2) = 0.0;
	KDL::JntArray q_out(chain.getNrOfJoints());

	hexapod_msgs::LegJointsState leg_msg;
	sensor_msgs::JointState joint_msg;
	geometry_msgs::PoseStamped pose_msg;
	int ik_valid;
	double x_0 = 0.499453, y_0 = -0.526126, z_0 = 0.019181, x, y, z, fi;  //0.14208; -0.14555; -0.11145
	while (ros::ok()){
		if (fi > 6.28) fi = 0;
		// x = x_0 + 0.05 * cos(fi);
		// y = y_0-0.2;
		// z = z_0 + 0.05 * sin(fi);

		x = 0.5; y = -0.59; z = 0.1;
		
		KDL::Frame p_in (KDL::Vector(x, y, z));
		pose_msg.header.frame_id = "base_link";
		pose_msg.pose.position.x = x;
		pose_msg.pose.position.y = y;
		pose_msg.pose.position.z = z;

		pose_msg.pose.orientation.x = 0.0;
		pose_msg.pose.orientation.y = 0.0;
		pose_msg.pose.orientation.z = 0.0;
		pose_msg.pose.orientation.w = 1.0;
		target_pose_msg_pub.publish(pose_msg);

		ik_valid = ik_solver_pos -> CartToJnt(q_init, p_in, q_out);
		ROS_INFO_STREAM(chain.getNrOfJoints());
		if (ik_valid >= 0){
			ROS_INFO("\nJoint 1: %f\nJoint 2: %f\nJoint 3: %f\n", q_out(0),q_out(1),q_out(2));
			for (int i=0; i<3; i++){
				leg_msg.joint[i] = q_out(i);
			}
			leg_msg_pub.publish(leg_msg);
			joint_msg.name.push_back("coxa_joint_r1");
			joint_msg.position.push_back(q_out(0));
			joint_msg.name.push_back("femur_joint_r1");
			joint_msg.position.push_back(q_out(1));
			joint_msg.name.push_back("tibia_joint_r1");
			joint_msg.position.push_back(q_out(2));
			joint_msg_pub.publish(joint_msg);
			joint_msg.name.clear();
			joint_msg.position.clear();
		}
		else {
			ROS_ERROR("IK not found");
		}
		fi += 0.05;
		q_init = q_out;
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}