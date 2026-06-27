#include "hexapod_gait_kinematics/gait_kinematics.hpp"
#include <chrono>

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};

GaitKinematics::GaitKinematics() : Node("gait_kinematics") {
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("root_name_body", "leg_base");
    this->declare_parameter<std::string>("tip_name_body", "tip");
    this->declare_parameter<double>("trapezoid_low_radius", 0.1);
    this->declare_parameter<double>("trapezoid_high_radius", 0.1);
    this->declare_parameter<double>("trapezoid_h", 0.1);
    this->declare_parameter<double>("clearance", 0.15);
    this->declare_parameter<double>("duration_ripple", 1.5);
    this->declare_parameter<double>("duration_tripod", 1.0);
}

bool GaitKinematics::init() {
    std::string robot_desc_string;
    if (!this->get_parameter("robot_description", robot_desc_string) || robot_desc_string.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Could not load the xml from parameter: robot_description");
        return false;
    }

    this->get_parameter("root_name_body", root_name);
    this->get_parameter("tip_name_body", tip_name);
    this->get_parameter("trapezoid_low_radius", trap_low_r);
    this->get_parameter("trapezoid_high_radius", trap_high_r);
    this->get_parameter("trapezoid_h", trap_h);
    this->get_parameter("clearance", trap_z);
    this->get_parameter("duration_ripple", d_ripple);
    this->get_parameter("duration_tripod", d_tripod);

    if (!loadModel(robot_desc_string)) {
        RCLCPP_FATAL(this->get_logger(), "Could not load models!");
        return false;
    }

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    client = this->create_client<hexapod_msgs::srv::GetIKSolver>("get_ik");
    joints_pub = this->create_publisher<hexapod_msgs::msg::LegsJointsState>("joints_to_controller", 1);
    
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;
    gait_control_sub = this->create_subscription<hexapod_msgs::msg::GaitCommand>(
        "/teleop/gait_control", 1,
        std::bind(&GaitKinematics::teleopGaitCtrl, this, std::placeholders::_1),
        sub_options
    );

    leg_target_pose_viz_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/gait_target_pose", 1);

    gait.setTrapezoid(trap_low_r, trap_high_r, trap_h, trap_z);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(40),
        std::bind(&GaitKinematics::timerCallback, this),
        cb_group_
    );

    return true;
}

void GaitKinematics::timerCallback() {
    KDL::Vector* final_vector = nullptr;

    if (gait_command.cmd == gait_command.RUNRIPPLE) {
        final_vector = gait.RunRipple(frames.begin(), gait_command.fi, gait_command.scale,
                                      gait_command.alpha, d_ripple);
        if (callService(final_vector)) {
            joints_pub->publish(legs);
        }
    } else if (gait_command.cmd == gait_command.RUNTRIPOD) {
        final_vector = gait.RunTripod(frames.begin(), gait_command.fi, gait_command.scale,
                                      gait_command.alpha, d_tripod);
        if (callService(final_vector)) {
            joints_pub->publish(legs);
        }
    } else if (gait_command.cmd == gait_command.PAUSE) {
        gait.Pause();
    } else if (gait_command.cmd == gait_command.STOP) {
        gait.Stop();
    }
}

void GaitKinematics::teleopGaitCtrl(const hexapod_msgs::msg::GaitCommand::SharedPtr gait_cmd) {
    gait_command.cmd = gait_cmd->cmd;
    gait_command.fi = gait_cmd->fi;
    gait_command.alpha = gait_cmd->alpha;
    gait_command.scale = gait_cmd->scale;
}

bool GaitKinematics::callService(KDL::Vector* vector) {
    auto request = std::make_shared<hexapod_msgs::srv::GetIKSolver::Request>();
    hexapod_msgs::msg::LegPositionState leg_pos_buf;
    geometry_msgs::msg::PoseArray target_pose_array;
    geometry_msgs::msg::Pose target_pose;

    for (int i = 0; i < num_legs; i++) {
        request->leg_number.push_back(i);
        leg_pos_buf.x = vector[i].x();
        leg_pos_buf.y = vector[i].y();
        leg_pos_buf.z = vector[i].z();

        target_pose_array.header.frame_id = "base_link";
        target_pose_array.header.stamp = this->now();
        
        target_pose.position.x = leg_pos_buf.x;
        target_pose.position.y = leg_pos_buf.y;
        target_pose.position.z = leg_pos_buf.z;
        target_pose.orientation.x = 0;
        target_pose.orientation.y = 0;
        target_pose.orientation.z = 0;
        target_pose.orientation.w = 1;

        target_pose_array.poses.push_back(target_pose);

        request->target_position.push_back(leg_pos_buf);
        request->current_joints.push_back(legs.joints_state[i]);
    }
    leg_target_pose_viz_pub->publish(target_pose_array);

    if (!client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available");
        return false;
    }

    auto result = client->async_send_request(request);
    std::future_status status = result.wait_for(std::chrono::seconds(2));
    if (status == std::future_status::ready) {
        auto response = result.get();
        if (response->error_codes == hexapod_msgs::srv::GetIKSolver::Response::IK_FOUND) {
            for (int i = 0; i < num_legs; i++) {
                for (int j = 0; j < num_joints; j++) {
                    legs.joints_state[i].joint[j] = response->target_joints[i].joint[j];
                }
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "An IK solution could not be found");
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service or service timed out");
        return false;
    }
    return true;
}

bool GaitKinematics::loadModel(const std::string xml) {
    KDL::Tree tree;

    if (!kdl_parser::treeFromString(xml, tree)) {
        RCLCPP_ERROR(this->get_logger(), "Could not initialize tree object");
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Construct tree");

    std::map<std::string, KDL::TreeElement>::const_iterator segments_iter;
    std::string link_name_result;
    for (int i = 0; i < num_legs; i++) {
        link_name_result = root_name + suffixes[i];
        segments_iter = tree.getSegment(link_name_result);
        frames.push_back((*segments_iter).second.segment.getFrameToTip());
    }
    for (int i = 0; i < num_legs; i++) {
        link_name_result = tip_name + suffixes[i];
        segments_iter = tree.getSegment(link_name_result);
        frames.push_back((*segments_iter).second.segment.getFrameToTip());
    }
    RCLCPP_INFO(this->get_logger(), "Get frames");

    for (int i = 0; i < num_legs; i++) {
        frames[i] = frames[i] * frames[i + num_legs] * KDL::Frame(KDL::Vector(0.5, 0, 0));
    }
    frames.resize(num_legs);

    for (int i = 0; i < num_legs; i++) {
        for (int j = 0; j < num_joints; j++) {
            legs.joints_state[i].joint[j] = 0;
        }
    }

    return true;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GaitKinematics>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize gait node");
        return -1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
