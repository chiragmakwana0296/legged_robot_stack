#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointStatesToJointTrajectoryNode(Node):
    def __init__(self):
        super().__init__("joint_state_to_trajectory_node")
        self.get_logger().info("Starting JointStatesToJointTrajectoryNode as joint_state_to_trajectory_node.")
        
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, 
            "/joint_group_position_controller/command", 
            1
        )
        
        self.joint_sub = self.create_subscription(
            JointState, 
            "/joint_states", 
            self.joint_state_cb, 
            1
        )

    def joint_state_cb(self, msg):
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory_msg.joint_names = msg.name
        
        point = JointTrajectoryPoint()
        point.time_from_start = rclpy.duration.Duration(seconds=1.0/60.0).to_msg()
        point.positions = list(msg.position)
        
        joint_trajectory_msg.points.append(point)
        self.joint_trajectory_pub.publish(joint_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatesToJointTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
