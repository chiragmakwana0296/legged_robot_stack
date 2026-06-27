#include "hexapod_kinematics/leg_ik_service.hpp"
#include <urdf/model.h>

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
const static double joint_upper_limit[6] = { 1.10,  0.9,  0.0, 0.5, 0.5, 0.5};
const static double joint_lower_limit[6] = {-1.10, -0.8, -1.5, -0.5, -0.5, -0.5};

LegKinematics::LegKinematics() : Node("leg_ik_service") {
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("root_name", "base_link");
    this->declare_parameter<std::string>("tip_name", "tip");
    this->declare_parameter<int>("maxIterations", 1000);
    this->declare_parameter<double>("epsilon", 0.01);
}

bool LegKinematics::init() {
    std::string robot_desc_string;
    if (!this->get_parameter("robot_description", robot_desc_string) || robot_desc_string.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Could not load the xml from parameter: robot_description");
        return false;
    }

    this->get_parameter("root_name", root_name);
    this->get_parameter("tip_name", tip_name);
    RCLCPP_INFO_STREAM(this->get_logger(), root_name);
    RCLCPP_INFO_STREAM(this->get_logger(), tip_name);

    if (!loadModel(robot_desc_string)) {
        RCLCPP_FATAL(this->get_logger(), "Could not load models!");
        return false;
    }

    int maxIterations;
    double epsilon;
    this->get_parameter("maxIterations", maxIterations);
    this->get_parameter("epsilon", epsilon);

    for (unsigned int i=0; i<num_legs; i++){
        fk_solver[i] = new KDL::ChainFkSolverPos_recursive(*chains_ptr[i]);
        ik_solver_vel[i] = new KDL::ChainIkSolverVel_pinv(*chains_ptr[i]);
        ik_solver_pos[i] = new KDL::ChainIkSolverPos_NR(*chains_ptr[i], *fk_solver[i], *ik_solver_vel[i], maxIterations, epsilon);
    }

    RCLCPP_INFO(this->get_logger(), "Advertising service");
    ik_service = this->create_service<hexapod_msgs::srv::GetIKSolver>(
        "get_ik",
        std::bind(&LegKinematics::getLegIKSolver, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Ready to client's request...");
    return true;
}

bool LegKinematics::loadModel(const std::string xml) {
    KDL::Tree tree;
    KDL::Chain chain;
    std::string tip_name_result;

    if (!kdl_parser::treeFromString(xml, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize tree object");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Construct tree");

    for (unsigned int i=0; i<num_legs; i++){
        tip_name_result = tip_name + suffixes[i];
        if (!tree.getChain(root_name, tip_name_result, chain)) {
            RCLCPP_ERROR(this->get_logger(), "Could not initialize chain_%s object", suffixes[i].c_str());
            return false;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), tip_name_result);

        chains_ptr[i] = new KDL::Chain(chain);
        RCLCPP_INFO_STREAM(this->get_logger(), "chains_ptr[i]->getNrOfSegments() " << chains_ptr[i]->getNrOfSegments());
        RCLCPP_INFO_STREAM(this->get_logger(), "chains_ptr[i]->getNrOfJoints() "<< chains_ptr[i]->getNrOfJoints());
    }
    RCLCPP_INFO(this->get_logger(), "Construct chains");

    return true;
}

void LegKinematics::getLegIKSolver(
    const std::shared_ptr<hexapod_msgs::srv::GetIKSolver::Request> request,
    std::shared_ptr<hexapod_msgs::srv::GetIKSolver::Response> response) {

    hexapod_msgs::msg::LegPositionState leg_dest_pos;
    response->target_joints.clear();

    for (size_t i = 0; i < request->leg_number.size(); i++) {
        leg_dest_pos = request->target_position[i];
        RCLCPP_INFO_STREAM(this->get_logger(), "request->target_position[i] "<< i << " x: " << leg_dest_pos.x << " y: " << leg_dest_pos.y << " z: " << leg_dest_pos.z);
        RCLCPP_INFO_STREAM(this->get_logger(), "request->leg_number.size() "<<  request->leg_number.size());
        RCLCPP_INFO_STREAM(this->get_logger(), "num_joints "<<  num_joints);

        KDL::JntArray jnt_pos_in(num_joints);
        KDL::JntArray jnt_pos_out(num_joints);

        for (unsigned int j=0; j < num_joints; j++) {
            jnt_pos_in(j) = request->current_joints[i].joint[j];
            RCLCPP_INFO_STREAM(this->get_logger(), "jnt_pos_in" << jnt_pos_in(j));
        }
        KDL::Frame F_dest(KDL::Vector(leg_dest_pos.x, leg_dest_pos.y, leg_dest_pos.z));

        int ik_valid = ik_solver_pos[request->leg_number[i]]->CartToJnt(jnt_pos_in, F_dest, jnt_pos_out);
        RCLCPP_INFO_STREAM(this->get_logger(), "ik_valid  " << ik_valid);
        
        if (ik_valid >= 0) {
            hexapod_msgs::msg::LegJointsState jnt_buf;
            for (unsigned int j=0; j<num_joints; j++) {
                jnt_buf.joint[j] = jnt_pos_out(j);
            }
            response->target_joints.push_back(jnt_buf);
            response->error_codes = hexapod_msgs::srv::GetIKSolver::Response::IK_FOUND;
            RCLCPP_DEBUG(this->get_logger(), "IK Solution for leg%s found", suffixes[request->leg_number[i]].c_str());
        } else {
            response->error_codes = hexapod_msgs::srv::GetIKSolver::Response::IK_NOT_FOUND;
            RCLCPP_ERROR(this->get_logger(), "An IK solution could not be found for leg%s", suffixes[request->leg_number[i]].c_str());
            return;
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LegKinematics>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize kinematics node");
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
