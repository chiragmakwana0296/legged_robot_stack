#include "rclcpp/rclcpp.hpp"
#include "hexapod_msgs/msg/leg_joints_state.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <eigen3/Eigen/Core>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <urdf/model.h>

void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // msg->position
    (void)msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("leg_kinematics_pub");

    KDL::Tree tree;
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive* fk_solver;
    KDL::ChainIkSolverVel_pinv* ik_solver_vel;
    KDL::ChainIkSolverPos_NR* ik_solver_pos;

    auto leg_msg_pub = node->create_publisher<hexapod_msgs::msg::LegJointsState>("leg_states", 1);
    auto joint_msg_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    auto target_pose_msg_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 1);

    rclcpp::Rate loop_rate(30);

    node->declare_parameter<std::string>("robot_description", "");
    std::string robot_desc_string;
    if (!node->get_parameter("robot_description", robot_desc_string) || robot_desc_string.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get robot_description parameter");
        return -1;
    }

    if (!kdl_parser::treeFromString(robot_desc_string, tree)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to construct kdl tree");
        return -1;
    }
    if (!tree.getChain("base_link", "tip_r1", chain)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to construct kdl chain");
        return -1;
    }
    RCLCPP_INFO(node->get_logger(), "Construct kdl chain");

    fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
    ik_solver_vel = new KDL::ChainIkSolverVel_pinv(chain);

    KDL::JntArray joint_min(chain.getNrOfJoints());
    KDL::JntArray joint_max(chain.getNrOfJoints());
    joint_min(0) = -3.14159; joint_min(1) = -3.14159; joint_min(2) = -3.14159;
    joint_max(0) = 3.14159;  joint_max(1) = 3.14159;  joint_max(2) = 3.14159;
    ik_solver_pos = new KDL::ChainIkSolverPos_NR(chain, *fk_solver, *ik_solver_vel, 1000, 0.01);
    
    RCLCPP_INFO_STREAM(node->get_logger(), chain.getNrOfJoints());
    KDL::JntArray q_init(chain.getNrOfJoints());
    q_init(0) = 0.0; q_init(1) = 0.0; q_init(2) = 0.0;
    KDL::JntArray q_out(chain.getNrOfJoints());

    hexapod_msgs::msg::LegJointsState leg_msg;
    sensor_msgs::msg::JointState joint_msg;
    geometry_msgs::msg::PoseStamped pose_msg;
    int ik_valid;
    double x, y, z, fi = 0;
    
    while (rclcpp::ok()) {
        if (fi > 6.28) fi = 0;

        x = 0.5; y = -0.59; z = 0.1;

        KDL::Frame p_in(KDL::Vector(x, y, z));
        pose_msg.header.frame_id = "base_link";
        pose_msg.pose.position.x = x;
        pose_msg.pose.position.y = y;
        pose_msg.pose.position.z = z;

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 1.0;
        target_pose_msg_pub->publish(pose_msg);

        ik_valid = ik_solver_pos->CartToJnt(q_init, p_in, q_out);
        if (ik_valid >= 0) {
            RCLCPP_INFO(node->get_logger(), "\nJoint 1: %f\nJoint 2: %f\nJoint 3: %f\n", q_out(0), q_out(1), q_out(2));
            for (int i=0; i<3; i++) {
                leg_msg.joint[i] = q_out(i);
            }
            leg_msg_pub->publish(leg_msg);
            
            joint_msg.header.stamp = node->now();
            joint_msg.name.push_back("coxa_joint_r1");
            joint_msg.position.push_back(q_out(0));
            joint_msg.name.push_back("femur_joint_r1");
            joint_msg.position.push_back(q_out(1));
            joint_msg.name.push_back("tibia_joint_r1");
            joint_msg.position.push_back(q_out(2));
            joint_msg_pub->publish(joint_msg);
            
            joint_msg.name.clear();
            joint_msg.position.clear();
        } else {
            RCLCPP_ERROR(node->get_logger(), "IK not found");
        }
        
        fi += 0.05;
        q_init = q_out;
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    
    rclcpp::shutdown();
    return 0;
}