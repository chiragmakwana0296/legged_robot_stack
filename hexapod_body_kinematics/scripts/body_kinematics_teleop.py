#!/usr/bin/env python
import rospy
from hexapod_msgs.srv import GetIKSolver,GetIKSolverRequest
from hexapod_msgs.msg  import BodyState
from geometry_msgs.msg import Twist 


class BodyIkTeleop():
    def __init__(self):
        rospy.init_node("body_kinematics_teleop_node")
        rospy.Subscriber("/cmd_vel/body", Twist, self.twist_linear_cb)
        # rospy.Subscriber("/cmd_vel/angular", Twist, self.twist_angular_cb)
        self.body_kinematics_pub = rospy.Publisher("/teleop/move_body", BodyState, queue_size=1)
        
        self.twist_linear_msg = Twist()
        self.twist_linear_msg = Twist()
        self.update = False

    def twist_linear_cb(self, msg):
        self.twist_linear_msg = msg
        self.update = True


    def twist_angular_cb(self, msg):
        self.twist_angular_msg = msg

        
    def main(self):
        while not rospy.is_shutdown():
            lim_max_x = 0.5
            lim_max_y = 0.5
            lim_max_z = 0.5
            lim_max_roll = 1.57/2.0
            lim_max_pitch = 1.57/2.0
            lim_max_yaw = 1.57/2.0
            command = BodyState()
            if self.update:
                command.x = lim_max_x * self.twist_linear_msg.linear.x
                command.y = lim_max_y * self.twist_linear_msg.linear.y
                command.z = lim_max_z * self.twist_linear_msg.linear.z
                command.roll  = lim_max_roll  * self.twist_linear_msg.angular.x
                command.pitch = lim_max_pitch * self.twist_linear_msg.angular.y
                command.yaw   = lim_max_yaw   * self.twist_linear_msg.angular.z
                command.leg_radius = 0.5
                self.body_kinematics_pub.publish(command)
                self.update = False



if __name__=="__main__":
    node = BodyIkTeleop()
    node.main()