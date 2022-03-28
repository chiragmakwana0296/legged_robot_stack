#ifndef HEXAPOD_BODY_KINEMATICS__INTERACTIVE_MARKER_HPP_
#define HEXAPOD_BODY_KINEMATICS__INTERACTIVE_MARKER_HPP_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>


class IntractiveMarker
{
private:
    ros::NodeHandle nh_;

    

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void createMarker();

    
public:
    IntractiveMarker(ros::NodeHandle* nodehandle);
};




#endif /* HEXAPOD_BODY_KINEMATICS__INTRACTIVE_MARKER_HPP_ */