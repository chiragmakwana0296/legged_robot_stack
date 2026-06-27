#include "hexapod_body_kinematics/interactive_marker.hpp"
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>

InteractiveMarker::InteractiveMarker() : Node("interactive_marker") {
    body_marker_server = std::make_unique<interactive_markers::InteractiveMarkerServer>("body_marker_server", this);
    createMarker();
}

void InteractiveMarker::processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
    (void)feedback;
}

void InteractiveMarker::createMarker() {
    visualization_msgs::msg::InteractiveMarker body_marker;
    body_marker.header.frame_id = "base_link";
    body_marker.header.stamp = this->now();
    body_marker.name = "body_marker";
    body_marker.description = "Body Marker";

    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE;
    sphere_marker.scale.x = 0.45;
    sphere_marker.scale.y = 0.45;
    sphere_marker.scale.z = 0.45;
    sphere_marker.color.r = 0.5;
    sphere_marker.color.g = 0.5;
    sphere_marker.color.b = 0.5;
    sphere_marker.color.a = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.markers.push_back(sphere_marker);

    // add the control to the interactive marker
    body_marker.controls.push_back(sphere_control);

    visualization_msgs::msg::InteractiveMarkerControl rotate_control;
    rotate_control.name = "move_3d";
    rotate_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

    // add the control to the interactive marker
    body_marker.controls.push_back(rotate_control);

    body_marker_server->insert(body_marker);
    body_marker_server->setCallback(body_marker.name, std::bind(&InteractiveMarker::processFeedback, this, std::placeholders::_1));
    body_marker_server->applyChanges();
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractiveMarker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
