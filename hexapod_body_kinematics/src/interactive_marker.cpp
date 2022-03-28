#include "hexapod_body_kinematics/interactive_marker.hpp"


IntractiveMarker::IntractiveMarker(ros::NodeHandle* nodehandle):nh_(*nodehandle){
    IntractiveMarker::createMarker();
}


void IntractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ){

}


void IntractiveMarker::createMarker(){
    visualization_msgs::InteractiveMarker body_marker;
    body_marker.header.frame_id = "base_link";
    body_marker.header.stamp=ros::Time::now();
    body_marker.name = "body_marker";
    body_marker.description = "Body Marker";

    
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.45;
    sphere_marker.scale.y = 0.45;
    sphere_marker.scale.z = 0.45;
    sphere_marker.color.r = 0.5;
    sphere_marker.color.g = 0.5;
    sphere_marker.color.b = 0.5;
    sphere_marker.color.a = 1.0;

    visualization_msgs::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.markers.push_back( sphere_marker );

    // add the control to the interactive marker
    body_marker.controls.push_back( sphere_control );

    visualization_msgs::InteractiveMarkerControl rotate_control;
    rotate_control.name = "move_3d";
    rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

    // add the control to the interactive marker
    body_marker.controls.push_back(rotate_control);

    // interactive_markers::InteractiveMarkerServer *body_marker_server->setCallback(body_marker.name, boost::bind(&IntractiveMarker::processFeedback, this, _1));
    // body_marker_server->insert(body_marker);
    // body_marker_server->applyChanges();
}



int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "exampleRosClass"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    IntractiveMarker intractiveMarker(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use

    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();
    return 0;
} 

