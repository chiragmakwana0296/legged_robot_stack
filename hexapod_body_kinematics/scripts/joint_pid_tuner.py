#!/usr/bin/env python
from pytest import param
import rospy
import dynamic_reconfigure.client
from hexapod_msgs.srv import SetJointPID, SetJointPIDRequest, SetJointPIDResponse

leg_suffix = ["_l1", "_l2", "_l3", "_r1", "_r2", "_r3"]
joint_names = ["joint_base", "joint_femur", "joint_tibia", "tibia_foot_joint"]
# joint_base_r1         
# joint_femur_r1        
# joint_tibia_r1        
# tibia_foot_joint_r1   
# leg_tip_inter_joint_r1
# leg_tip_joint_r1      
class ServiceSetJointPID:
    def __init__(self):
        rospy.init_node("service_joint_pid_tuner_node")
        rospy.loginfo("Starting ServiceSetJointPID as service_joint_pid_tuner_node.")
        client = dynamic_reconfigure.client.Client("gazebo", timeout=30, config_callback=self.callback)
        joine_pid_srv = rospy.Service("hexapod/set_joint_pid", SetJointPID, self.joine_pid_cb)
    
    def callback(self, config):
        rospy.loginfo(config)

    def joine_pid_cb(self, req):
        param = rospy.get_param("/gazebo_ros_control")
        print(param)
        for joint in joint_names:
            for leg in leg_suffix:
                rospy.set_param("/gazebo_ros_control/pid_gains/{0}{1}/p".format(joint, leg), req.p)
                rospy.set_param("/gazebo_ros_control/pid_gains/{0}{1}/i".format(joint, leg), req.i)
                rospy.set_param("/gazebo_ros_control/pid_gains/{0}{1}/d".format(joint, leg), req.d)
                
                rospy.set_param("/joint_group_position_controller/gains/{0}{1}/p".format(joint, leg), req.p)
                rospy.set_param("/joint_group_position_controller/gains/{0}{1}/i".format(joint, leg), req.i)
                rospy.set_param("/joint_group_position_controller/gains/{0}{1}/d".format(joint, leg), req.d)
        return SetJointPIDResponse(success=True)

if __name__ == "__main__":
    service_joint_pid_tuner_node = ServiceSetJointPID()
    rospy.spin()