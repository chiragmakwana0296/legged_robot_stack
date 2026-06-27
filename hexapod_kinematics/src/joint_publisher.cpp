#include "rclcpp/rclcpp.hpp"
#include "hexapod_msgs/msg/legs_joints_state.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

class HexapodJointPublisher : public rclcpp::Node {
public:
    HexapodJointPublisher() : Node("hexapod_joint_publisher") {
        joint_msg_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
        sub = this->create_subscription<hexapod_msgs::msg::LegsJointsState>(
            "joints_to_controller", 1,
            std::bind(&HexapodJointPublisher::chatterLegsState, this, std::placeholders::_1)
        );
    }

private:
    void chatterLegsState(const hexapod_msgs::msg::LegsJointsState::SharedPtr state) {
        std::string joint_name;
        joint_msg.header.stamp = this->now();
        
        for (int suf=0; suf<6; suf++) {
            for (int name=0; name<6; name++) {
                joint_name = names[name] + suffixes[suf];
                joint_msg.name.push_back(joint_name);
                joint_msg.position.push_back(state->joints_state[suf].joint[name]);
            }
        }
        
        joint_msg_pub->publish(joint_msg);
        joint_msg.name.clear();
        joint_msg.position.clear();
    }

    const std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};
    const std::string names[6] = {"joint_base", "joint_femur", "joint_tibia", "tibia_foot_joint", "leg_tip_inter_joint", "leg_tip_joint"};
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_msg_pub;
    rclcpp::Subscription<hexapod_msgs::msg::LegsJointsState>::SharedPtr sub;
    sensor_msgs::msg::JointState joint_msg;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HexapodJointPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
