#include "hexapod_gait_kinematics/gait_kinematics.hpp"

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};


bool GaitKinematics::init() {
	std::string robot_desc_string;
	// Get URDF XML
	if (!node.getParam("robot_description", robot_desc_string)) {
		   ROS_FATAL("Could not load the xml from parameter: robot_description");
		   return false;
	   }

	// Get Root and Tip From Parameter Server
	node.param("root_name_body", root_name, std::string("leg_base"));
	node.param("tip_name_body", tip_name, std::string("tip"));

	// Get Gait Settings From Parameter Server
	node.param("trapezoid_low_radius", trap_low_r, 0.1);
	node.param("trapezoid_high_radius", trap_high_r, 0.1);
	node.param("trapezoid_h", trap_h, 0.1);
	node.param("clearance", trap_z, 0.15);
	node.param("duration_ripple", d_ripple, 1.5);
	node.param("duration_tripod", d_tripod, 1.0);

	// Load and Read Models
	if (!loadModel(robot_desc_string)) {
		ROS_FATAL("Could not load models!");
		return false;
	}

	client = node.serviceClient<hexapod_msgs::GetIKSolver>("/hexapod_leg_kinematics/get_ik");
	joints_pub = node.advertise<hexapod_msgs::LegsJointsState>("joints_to_controller", 1);
	gait_control_sub = node.subscribe<hexapod_msgs::GaitCommand>("/teleop/gait_control", 1, &GaitKinematics::teleopGaitCtrl, this);
	leg_target_pose_viz_pub = node.advertise<geometry_msgs::PoseArray>("/gait_target_pose", 1);
	
	return true;
}


void GaitKinematics::gaitGenerator(){

	KDL::Vector* final_vector;
	Gait gait;
	gait.setTrapezoid(trap_low_r, trap_high_r, trap_h, trap_z);

	while (node.ok()){
		if (gait_command.cmd == gait_command.RUNRIPPLE){
			final_vector = gait.RunRipple(frames.begin(), gait_command.fi, gait_command.scale,
											gait_command.alpha, d_ripple);
			if (callService(final_vector)){
				joints_pub.publish(legs);
			}
		}

		else if (gait_command.cmd == gait_command.RUNTRIPOD){
			final_vector = gait.RunTripod(frames.begin(), gait_command.fi, gait_command.scale,
											gait_command.alpha, d_tripod);
			if (callService(final_vector)){
				joints_pub.publish(legs);
			}
		}

		else if (gait_command.cmd == gait_command.PAUSE){
			gait.Pause();
		}

		else if (gait_command.cmd == gait_command.STOP){
			gait.Stop();
		}
		ros::spinOnce();
		ros::Rate(25).sleep();
	}

}


void GaitKinematics::teleopGaitCtrl (const hexapod_msgs::GaitCommandConstPtr &gait_cmd){
	gait_command.cmd = gait_cmd->cmd;
	gait_command.fi = gait_cmd->fi;
	gait_command.alpha = gait_cmd->alpha;
	gait_command.scale = gait_cmd->scale;

}


bool GaitKinematics::callService (KDL::Vector* vector){
	hexapod_msgs::LegPositionState leg_pos_buf;
	srv.request.leg_number.clear();
	srv.request.target_position.clear();
	srv.request.current_joints.clear();
	geometry_msgs::PoseArray target_pose_array;
	geometry_msgs::Pose target_pose;
	//Creating message to request
	for (int i=0; i<num_legs; i++){
		srv.request.leg_number.push_back(i);
		leg_pos_buf.x = vector[i].x();
		leg_pos_buf.y = vector[i].y();
		leg_pos_buf.z = vector[i].z();

		// Visualization
		target_pose_array.header.frame_id = "base_link";
		target_pose_array.header.stamp = ros::Time::now();;
		
		target_pose.position.x = leg_pos_buf.x;
		target_pose.position.y = leg_pos_buf.y;
		target_pose.position.z = leg_pos_buf.z;
		target_pose.orientation.x = 0;
		target_pose.orientation.y = 0;
		target_pose.orientation.z = 0;
		target_pose.orientation.w = 1;

		target_pose_array.poses.push_back(target_pose);

		srv.request.target_position.push_back(leg_pos_buf);
		srv.request.current_joints.push_back(legs.joints_state[i]);
	}
	leg_target_pose_viz_pub.publish(target_pose_array);
	//Call service and parsing response
	if (client.call(srv)){
		if (srv.response.error_codes==srv.response.IK_FOUND){
			for (int i=0; i<num_legs; i++){
				for (int j = 0; j < num_joints; j++) {
						legs.joints_state[i].joint[j] = srv.response.target_joints[i].joint[j];
				}
//				ROS_DEBUG("Joints received leg%s\t1: %f\t2: %f\t3: %f", suffixes[i].c_str(),
//							legs.joints_state[i].joint[0],
//							legs.joints_state[i].joint[1],
//							legs.joints_state[i].joint[2]);
			}
		}
		else {
			ROS_ERROR("An IK solution could not be found");
			return 0;
		}
	}
	else {
		ROS_ERROR("Failed to call service");
		return 0;
	}
	return true;
}


bool GaitKinematics::loadModel(const std::string xml){
	//Construct tree with kdl_parser
	KDL::Tree tree;

	if (!kdl_parser::treeFromString(xml, tree)) {
		ROS_ERROR("Could not initialize tree object");
		return false;
	}
	ROS_INFO("Construct tree");

	//Get coxa and leg_center frames via segments (for calculating vectors)
	std::map<std::string,KDL::TreeElement>::const_iterator segments_iter;
	std::string link_name_result;
	for (int i=0; i<num_legs; i++){
		link_name_result = root_name + suffixes[i];
		segments_iter = tree.getSegment(link_name_result);
		frames.push_back((*segments_iter).second.segment.getFrameToTip());
	}
	for (int i=0; i<num_legs; i++){
		link_name_result = tip_name + suffixes[i];
		segments_iter = tree.getSegment(link_name_result);
		frames.push_back((*segments_iter).second.segment.getFrameToTip());
	}
	ROS_INFO("Get frames");

	//Vector iterators
	for (int i=0; i<num_legs; i++){
		frames[i] = frames[i] * frames[i+num_legs] * KDL::Frame (KDL::Vector (0.5,0,0));  //!!!!!!!!!!!!!!!!!
	}
	frames.resize(num_legs);

	for (int i=0; i<num_legs; i++){
		for (int j = 0; j < num_joints; j++) {
			legs.joints_state[i].joint[j] = 0;
		}
	}

	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "gait_kinematics");
	GaitKinematics g;
    if (g.init()<0) {
        ROS_ERROR("Could not initialize gait node");
        return -1;
    }
    g.gaitGenerator();

    ros::spin();
    return 0;
}
