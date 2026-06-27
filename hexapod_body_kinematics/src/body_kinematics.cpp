#include "hexapod_body_kinematics/body_kinematics.hpp"
#include <chrono>

BodyKinematics::BodyKinematics() : Node("body_kinematics") {
    this->declare_parameter<std::string>("robot_description", "");
    this->declare_parameter<std::string>("root_name_body", "leg_base");
    this->declare_parameter<std::string>("tip_name_body", "tip");
    this->declare_parameter<double>("clearance", 0.045);
}

const static std::string suffixes[6] = {"_r1", "_r2", "_r3", "_l1", "_l2", "_l3"};

bool BodyKinematics::init() {
    std::string robot_desc_string;
    if (!this->get_parameter("robot_description", robot_desc_string) || robot_desc_string.empty()) {
        RCLCPP_FATAL(this->get_logger(), "Could not load the xml from parameter: robot_description");
        return false;
    }

    this->get_parameter("root_name_body", root_name);
    this->get_parameter("tip_name_body", tip_name);
    this->get_parameter("clearance", z);

    if (!loadModel(robot_desc_string)) {
        RCLCPP_FATAL(this->get_logger(), "Could not load models!");
        return false;
    }

    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    client = this->create_client<hexapod_msgs::srv::GetIKSolver>("get_ik");
    joints_pub = this->create_publisher<hexapod_msgs::msg::LegsJointsState>("/joints_to_controller", 1);
    
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = cb_group_;

    body_move_sub = this->create_subscription<hexapod_msgs::msg::BodyState>(
        "/teleop/move_body", 1,
        std::bind(&BodyKinematics::teleopBodyMove, this, std::placeholders::_1),
        sub_options
    );

    body_cmd_sub = this->create_subscription<hexapod_msgs::msg::BodyCommand>(
        "/teleop/body_command", 1,
        std::bind(&BodyKinematics::teleopBodyCmd, this, std::placeholders::_1),
        sub_options
    );

    leg_target_pose_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/legs_target_pose", 1);

    bs.leg_radius = 0.5;
    bs.z = 0.2;

    startup_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
            startup_timer_->cancel();
            if (calculateKinematics(&bs)) {
                joints_pub->publish(legs);
            }
        },
        cb_group_
    );

    RCLCPP_INFO(this->get_logger(), "Ready to receive teleop messages... ");

    return true;
}

bool BodyKinematics::loadModel(const std::string xml) {
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
        frames[i] = frames[i] * frames[i + num_legs];
    }
    frames.resize(num_legs);

    for (int i = 0; i < num_legs; i++) {
        for (int j = 0; j < num_joints; j++) {
            legs.joints_state[i].joint[j] = 0;
        }
    }

    return true;
}

bool BodyKinematics::calculateKinematics(hexapod_msgs::msg::BodyState* body_ptr) {
    rotation = KDL::Rotation::RPY(body_ptr->roll, body_ptr->pitch, body_ptr->yaw);
    geometry_msgs::msg::PoseArray target_pose_array;
    geometry_msgs::msg::Pose target_pose;
    
    femur_frame = KDL::Frame(KDL::Vector(body_ptr->leg_radius, 0, 0));
    offset_vector = KDL::Vector(body_ptr->x, body_ptr->y, body_ptr->z);
    rotate_correction = KDL::Vector(body_ptr->z * tan(body_ptr->pitch), -(body_ptr->z * tan(body_ptr->roll)), 0);

    for (int i = 0; i < num_legs; i++) {
        tibia_foot_frame = frames[i] * femur_frame;
        final_vector[i] = (rotation * tibia_foot_frame.p) + offset_vector + rotate_correction;
        RCLCPP_INFO_STREAM(this->get_logger(), "Position vector leg: " << suffixes[i].c_str() << "  x: " << final_vector[i](0) << "  y: " << final_vector[i](1) << "  z:  " << final_vector[i](2));

        target_pose_array.header.frame_id = "base_link";
        target_pose_array.header.stamp = this->now();

        target_pose.position.x = final_vector[i](0);
        target_pose.position.y = final_vector[i](1);
        target_pose.position.z = final_vector[i](2);
        target_pose.orientation.x = 0;
        target_pose.orientation.y = 0;
        target_pose.orientation.z = 0;
        target_pose.orientation.w = 1;

        target_pose_array.poses.push_back(target_pose);
    }
    leg_target_pose_pub->publish(target_pose_array);

    if (!callService(final_vector)) {
        return false;
    }

    return true;
}

bool BodyKinematics::callService(KDL::Vector* vector) {
    auto request = std::make_shared<hexapod_msgs::srv::GetIKSolver::Request>();
    hexapod_msgs::msg::LegPositionState leg_pos_buf;

    for (int i = 0; i < num_legs; i++) {
        RCLCPP_INFO_STREAM(this->get_logger(), "num_legs  " << num_legs);
        request->leg_number.push_back(i);
        leg_pos_buf.x = vector[i].x();
        leg_pos_buf.y = vector[i].y();
        leg_pos_buf.z = vector[i].z();
        request->target_position.push_back(leg_pos_buf);
        request->current_joints.push_back(legs.joints_state[i]);
    }

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
                RCLCPP_DEBUG(this->get_logger(), "Joints received leg%s\t1: %f\t2: %f\t3: %f", suffixes[i].c_str(),
                             legs.joints_state[i].joint[0],
                             legs.joints_state[i].joint[1],
                             legs.joints_state[i].joint[2]);
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

void BodyKinematics::teleopBodyMove(const hexapod_msgs::msg::BodyState::SharedPtr body_state) {
    bs.x = body_state->x;
    bs.y = body_state->y;
    bs.z = body_state->z;
    bs.pitch = body_state->pitch;
    bs.roll = body_state->roll;
    bs.yaw = body_state->yaw;
    bs.leg_radius = body_state->leg_radius;
    if (calculateKinematics(&bs)) {
        joints_pub->publish(legs);
    }
}

void BodyKinematics::teleopBodyCmd(const hexapod_msgs::msg::BodyCommand::SharedPtr body_cmd) {
    if (body_cmd->cmd == body_cmd->STAND_UP_CMD) {
        RCLCPP_ERROR(this->get_logger(), "STAND_UP_CMD");
        rclcpp::Rate r(25);
        while (bs.z >= -0.2) {
            bs.z -= 0.0025;
            r.sleep();
            if (calculateKinematics(&bs)) {
                joints_pub->publish(legs);
            }
        }
    }
    if (body_cmd->cmd == body_cmd->SEAT_DOWN_CMD) {
        RCLCPP_ERROR(this->get_logger(), "SEAT_DOWN_CMD");
        rclcpp::Rate r(25);
        while (bs.z <= -0.016) {
            bs.z += 0.0025;
            r.sleep();
            if (calculateKinematics(&bs)) {
                joints_pub->publish(legs);
            }
        }
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BodyKinematics>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Could not initialize kinematics node");
        return -1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
