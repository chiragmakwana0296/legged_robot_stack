#ifndef HEXAPOD_BODY_KINEMATICS__INTERACTIVE_MARKER_HPP_
#define HEXAPOD_BODY_KINEMATICS__INTERACTIVE_MARKER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

class InteractiveMarker : public rclcpp::Node
{
private:
    std::unique_ptr<interactive_markers::InteractiveMarkerServer> body_marker_server;

    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback );
    void createMarker();

public:
    InteractiveMarker();
};

#endif /* HEXAPOD_BODY_KINEMATICS__INTERACTIVE_MARKER_HPP_ */