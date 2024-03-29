#include "ros/ros.h"
#include "hexapod_msgs/LegsJointsState.h"
#include <sensor_msgs/JointState.h>

typedef boost::shared_ptr<hexapod_msgs::LegsJointsState const> LegsStateConstPtr;
const std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const std::string names[6] = {"joint_base", "joint_femur", "joint_tibia", "tibia_foot_joint", "leg_tip_inter_joint", "leg_tip_joint"};
ros::Publisher joint_msg_pub;
sensor_msgs::JointState joint_msg;

void chatterLegsState (const LegsStateConstPtr& state){
	std::string joint_name;
	joint_msg.header.stamp = ros::Time::now();
	for(int suf=0; suf<6; suf++){
		for (int name=0; name<6; name++){
			joint_name = names[name] + suffixes[suf];
			joint_msg.name.push_back(joint_name.c_str());
			joint_msg.position.push_back(state->joints_state[suf].joint[name]);
		}
	}
	joint_msg_pub.publish(joint_msg);
	joint_msg.name.clear();
	joint_msg.position.clear();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexapod_joint_publisher");
	ros::NodeHandle node;

	joint_msg_pub = node.advertise<sensor_msgs::JointState>("joint_states", 1);
//	ros::Rate loop_rate(20);

	ros::Subscriber sub = node.subscribe("joints_to_controller", 1, chatterLegsState);

	ros::spin();

	return 0;
}
