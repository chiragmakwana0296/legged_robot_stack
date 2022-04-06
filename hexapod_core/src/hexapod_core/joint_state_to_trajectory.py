#!/usr/bin/env python
#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class JointStatesToJointTrajectoryNode:
    def __init__(self):
        rospy.init_node("joint_state_to_trajectory_node")
        rospy.loginfo("Starting JointStatesToJointTrajectoryNode as joint_state_to_trajectory_node.")
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.joint_trajectory_pub = rospy.Publisher("/joint_group_position_controller/command", JointTrajectory, queue_size=1)
        


    def joint_state_cb(self, msg):
        # msg = JointState()
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header.stamp = rospy.Time.now()
        joint_trajectory_msg.joint_names = msg.name
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(1.0/60.0)
        point.positions = [0]*len(msg.name)
        point.positions = msg.position
        joint_trajectory_msg.points.append(point)

        self.joint_trajectory_pub.publish(joint_trajectory_msg)
        
        

if __name__ == "__main__":
    joint_state_to_trajectory_node = JointStatesToJointTrajectoryNode()
    rospy.spin()
