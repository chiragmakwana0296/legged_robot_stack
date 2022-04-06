#include "hexapod_kinematics/leg_ik_service.hpp"

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const static double joint_upper_limit[6] = { 1.10,  0.9,  0.0, 3.14, 3.14, 3.14};
const static double joint_lower_limit[6] = {-1.10, -0.9, -1.5, -3.14, -3.14, -3.14};
LegKinematics::LegKinematics():	node_private("~"){}

bool LegKinematics::init() {
	 std::string robot_desc_string;
    // Get URDF XML
    if (!node.getParam("robot_description", robot_desc_string)) {
           ROS_FATAL("Could not load the xml from parameter: robot_description");
           return false;
    }
	// ROS_INFO_STREAM(robot_desc_string);
    // Get Root and Tip From Parameter Server
    node_private.param("root_name", root_name, std::string("base_link"));
    node_private.param("tip_name", tip_name, std::string("tip"));
	ROS_INFO_STREAM(root_name);
	ROS_INFO_STREAM(tip_name);
    // Load and Read Models
    if (!loadModel(robot_desc_string)) {
        ROS_FATAL("Could not load models!");
        return false;
    }

    // Get Min and Max joints limits
    // node.param("joint_lower_limit", joint_lower_limit, -(KDL::PI/2));
    // node.param("joint_upper_limit", joint_upper_limit, KDL::PI/2);
    // joint_min.resize(num_joints);
    // joint_max.resize(num_joints);
    // for (unsigned int i=0; i<num_joints; i++){
    // 	joint_min(i) = joint_lower_limit[i];
    // 	joint_max(i) = joint_upper_limit[i];
    // }
    // Get Solver Parameters
	int maxIterations;
	double epsilon;

	node_private.param("maxIterations", maxIterations, 1000);
	node_private.param("epsilon", epsilon, 0.01);

	// Build Solvers
	for (unsigned int i=0; i<num_legs; i++){
		  fk_solver[i] = new KDL::ChainFkSolverPos_recursive(*chains_ptr[i]);
		  ik_solver_vel[i] = new KDL::ChainIkSolverVel_pinv(*chains_ptr[i]);
		  ik_solver_pos[i] = new KDL::ChainIkSolverPos_NR(*chains_ptr[i], *fk_solver[i], *ik_solver_vel[i], maxIterations, epsilon);
	}

	ROS_INFO("Advertising service");
	ik_service = node_private.advertiseService("get_ik", &LegKinematics::getLegIKSolver,this);
	ROS_INFO("Ready to client's request...");
	return true;
}

bool LegKinematics::loadModel(const std::string xml) {
    KDL::Tree tree;
    KDL::Chain chain;
    std::string tip_name_result;

    if (!kdl_parser::treeFromString(xml, tree)) {
        ROS_ERROR("Could not initialize tree object");
        return false;
    }
    ROS_INFO("Construct tree");

    for (int i=0; i<num_legs; i++){
    	tip_name_result = tip_name + suffixes[i];
		if (!tree.getChain(root_name, tip_name_result, chain)) {
			ROS_ERROR("Could not initialize chain_%s object", suffixes[i].c_str());
			return false;
		}
		ROS_INFO_STREAM(tip_name_result);

		chains_ptr[i] = new KDL::Chain(chain);
		ROS_INFO_STREAM("chains_ptr[i]->getNrOfSegments() " << chains_ptr[i]->getNrOfSegments());
		ROS_INFO_STREAM("chains_ptr[i]->getNrOfJoints() "<< chains_ptr[i]->getNrOfJoints());
    }
    ROS_INFO("Construct chains");

    return true;
}

bool LegKinematics::getLegIKSolver (hexapod_msgs::GetIKSolver::Request &request, hexapod_msgs::GetIKSolver::Response &response){

	hexapod_msgs::LegPositionState leg_dest_pos;
	response.target_joints.clear();
	
	for (int i = 0; i < request.leg_number.size(); i++){
		leg_dest_pos = request.target_position[i];
		ROS_INFO_STREAM("request.target_position[i] "<< i << " " << leg_dest_pos);
		ROS_INFO_STREAM("request.leg_number.size() "<<  request.leg_number.size());
		ROS_INFO_STREAM("num_joints "<<  num_joints);

		KDL::JntArray jnt_pos_in(num_joints);
		KDL::JntArray jnt_pos_out(num_joints);

		//Get initial joints and frame
		for (unsigned int j=0; j < num_joints; j++) {
			jnt_pos_in(j) = request.current_joints[i].joint[j];
			ROS_INFO_STREAM("jnt_pos_in" <<jnt_pos_in(j));
		}
		KDL::Frame F_dest (KDL::Vector(leg_dest_pos.x, leg_dest_pos.y, leg_dest_pos.z));
		
		//IK solver
		int ik_valid = ik_solver_pos[request.leg_number[i]]>CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);
		ik_solver_vel[request.leg_number[i]]-
		ROS_INFO_STREAM("ik_valid  " <<ik_valid);
		if (ik_valid >= 0) {
			hexapod_msgs::LegJointsState jnt_buf;
			for (unsigned int j=0; j<num_joints; j++) {
					jnt_buf.joint[j] = jnt_pos_out(j);
				}
			response.target_joints.push_back(jnt_buf);
			response.error_codes = response.IK_FOUND;
			ROS_DEBUG("IK Solution for leg%s found", suffixes[request.leg_number[i]].c_str());
		}
		else {
			response.error_codes = response.IK_NOT_FOUND;
			ROS_ERROR("An IK solution could not be found for leg%s", suffixes[request.leg_number[i]].c_str());
			return true;
		}
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "leg_ik_service");
	LegKinematics k;
    if (k.init()<0) {
        ROS_ERROR("Could not initialize kinematics node");
        return -1;
    }
    ros::spin();
    return 0;
}
