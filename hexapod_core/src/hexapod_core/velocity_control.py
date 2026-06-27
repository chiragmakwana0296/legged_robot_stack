#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hexapod_msgs.msg import GaitCommand
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import math

PS3_BUTTON_SELECT               =  0
PS3_BUTTON_STICK_LEFT           =  1
PS3_BUTTON_STICK_RIGHT          =  2
PS3_BUTTON_START                =  3
PS3_BUTTON_CROSS_UP             =  4
PS3_BUTTON_CROSS_RIGHT          =  5
PS3_BUTTON_CROSS_DOWN           =  6
PS3_BUTTON_CROSS_LEFT           =  7
PS3_BUTTON_REAR_LEFT_2          =  8
PS3_BUTTON_REAR_RIGHT_2         =  9
PS3_BUTTON_REAR_LEFT_1          =  10
PS3_BUTTON_REAR_RIGHT_1         =  11
PS3_BUTTON_ACTION_TRIANGLE      =  12
PS3_BUTTON_ACTION_CIRCLE        =  13
PS3_BUTTON_ACTION_CROSS         =  14
PS3_BUTTON_ACTION_SQUARE        =  15
PS3_BUTTON_PAIRING              =  16

PS3_AXIS_STICK_LEFT_LEFTWARDS   =  0
PS3_AXIS_STICK_LEFT_UPWARDS     =  1
PS3_AXIS_STICK_RIGHT_LEFTWARDS  =  2
PS3_AXIS_STICK_RIGHT_UPWARDS    =  3
PS3_AXIS_BUTTON_CROSS_UP        =  4
PS3_AXIS_BUTTON_CROSS_RIGHT     =  5
PS3_AXIS_BUTTON_CROSS_DOWN      =  6
PS3_AXIS_BUTTON_CROSS_LEFT      =  7
PS3_AXIS_BUTTON_REAR_LEFT_2     =  8
PS3_AXIS_BUTTON_REAR_RIGHT_2    =  9
PS3_AXIS_BUTTON_REAR_LEFT_1     =  10
PS3_AXIS_BUTTON_REAR_RIGHT_1    =  11
PS3_AXIS_BUTTON_ACTION_TRIANGLE =  12
PS3_AXIS_BUTTON_ACTION_CIRCLE   =  13
PS3_AXIS_BUTTON_ACTION_CROSS    =  14
PS3_AXIS_BUTTON_ACTION_SQUARE   =  15
PS3_AXIS_ACCELEROMETER_LEFT     =  16
PS3_AXIS_ACCELEROMETER_FORWARD  =  17
PS3_AXIS_ACCELEROMETER_UP       =  18
PS3_AXIS_GYRO_YAW               =  19

axis_body_roll          = PS3_AXIS_STICK_LEFT_LEFTWARDS
axis_body_pitch         = PS3_AXIS_STICK_LEFT_UPWARDS
axis_body_yaw           = PS3_AXIS_STICK_RIGHT_LEFTWARDS
axis_body_y_off         = PS3_AXIS_STICK_LEFT_LEFTWARDS
axis_body_x_off         = PS3_AXIS_STICK_LEFT_UPWARDS
axis_body_z_off         = PS3_AXIS_STICK_RIGHT_UPWARDS
axis_fi_x               = PS3_AXIS_STICK_LEFT_LEFTWARDS
axis_fi_y               = PS3_AXIS_STICK_LEFT_UPWARDS
axis_alpha              = PS3_AXIS_STICK_RIGHT_LEFTWARDS
axis_scale              = PS3_AXIS_STICK_RIGHT_UPWARDS

button_left_shift       = PS3_BUTTON_REAR_LEFT_1
button_right_shift      = PS3_BUTTON_REAR_RIGHT_1
button_right_shift_2    = PS3_BUTTON_REAR_LEFT_2
button_start            = PS3_BUTTON_START
button_gait_switch      = PS3_BUTTON_ACTION_TRIANGLE
button_imu              = PS3_BUTTON_ACTION_CROSS

class HexapodVelocityControl(Node):
    def __init__(self):
        super().__init__("hexapod_velocity_control")
        self.joy_command_pub = self.create_publisher(Joy, "/joy_out", 1)
        self.joy_command_viz_pub = self.create_publisher(TwistStamped, "/joy_twist_viz", 1)
        
        self.create_subscription(Twist, "/cmd_vel/angular", self.cmd_vel_ang_cb, 1)
        self.create_subscription(Twist, "/cmd_vel/linear", self.cmd_vel_lin_cb, 1)
        self.create_subscription(Twist, "/cmd_vel/lin_ang_z", self.cmd_vel_lin_ang_z_cb, 1)
       
        self.create_subscription(Bool, "/btn/gait_switch", self.gait_switch_cb, 1)
        self.create_subscription(Bool, "/btn/left_shift", self.left_shift_cb, 1)
        self.create_subscription(Bool, "/btn/right_shift", self.right_shift_cb, 1)
        self.create_subscription(Bool, "/btn/start", self.start_cb, 1)

        self.create_subscription(Joy, "/joy", self.joy_cb, 1)
        
        self.btn_gait_switch_msg = int()
        self.btn_left_shift_msg = int()
        self.btn_right_shift_msg = int()
        self.btn_start_msg = int()

        self.twist_ang_msg = Twist()
        self.twist_lin_msg = Twist()
        self.twist_lin_ang_z_msg = Twist()

        self.joy_update = False
        self.timer = self.create_timer(0.04, self.timer_cb)

    def gait_switch_cb(self, msg):
        self.btn_gait_switch_msg = int(msg.data)
        self.joy_update = True

    def left_shift_cb(self, msg):
        self.btn_left_shift_msg = int(msg.data)
        self.joy_update = True

    def right_shift_cb(self, msg):
        self.btn_right_shift_msg = int(msg.data)
        self.joy_update = True

    def start_cb(self, msg):
        self.btn_start_msg = int(msg.data)
        self.joy_update = True

    def cmd_vel_ang_cb(self, msg):
        self.twist_ang_msg = msg
        self.joy_update = True

    def cmd_vel_lin_cb(self, msg):
        self.twist_lin_msg = msg
        self.joy_update = True
    
    def cmd_vel_lin_ang_z_cb(self, msg):
        self.twist_lin_ang_z_msg = msg
        self.joy_update = True
    
    def joy_cb(self, msg):
        joy_twist_viz_msg = TwistStamped()
        joy_msg = Joy()
        joy_msg.axes = [0.0]*20
        joy_msg.axes[axis_body_roll] =  float(msg.axes[0])
        joy_msg.axes[axis_body_pitch] = float(msg.axes[1])
        joy_msg.axes[axis_body_yaw  ] = float(msg.axes[4])

        joy_msg.axes[axis_body_x_off] = float(msg.axes[2])
        joy_msg.axes[axis_body_y_off] = float(msg.axes[3])
        joy_msg.axes[axis_body_z_off] = float(msg.axes[5])

        joy_msg.axes[axis_fi_x      ] = float(msg.axes[3])
        joy_msg.axes[axis_fi_y      ] = float(msg.axes[2])

        joy_msg.axes[axis_alpha     ] = float(msg.axes[0])
        joy_msg.axes[axis_scale     ] = float(msg.axes[1])

        joy_msg.buttons = [0]*17
        joy_msg.buttons[button_left_shift   ] = int(msg.buttons[4])
        joy_msg.buttons[button_right_shift  ] = int(msg.buttons[5])
        joy_msg.buttons[button_right_shift_2] = int(msg.buttons[7])
        joy_msg.buttons[button_start        ] = int(msg.buttons[9])
        joy_msg.buttons[button_gait_switch  ] = int(msg.buttons[8])

        self.joy_command_pub.publish(joy_msg)

        joy_twist_viz_msg.header.frame_id = "base_link"
        joy_twist_viz_msg.header.stamp = self.get_clock().now().to_msg()
        joy_twist_viz_msg.twist.linear.x = self.twist_lin_msg.linear.x
        joy_twist_viz_msg.twist.linear.y = self.twist_lin_msg.linear.y
        joy_twist_viz_msg.twist.linear.z = self.twist_lin_msg.linear.z
        joy_twist_viz_msg.twist.angular.x = self.twist_lin_msg.angular.x
        joy_twist_viz_msg.twist.angular.y = self.twist_lin_msg.angular.y
        joy_twist_viz_msg.twist.angular.z = self.twist_lin_msg.angular.z
        
        self.joy_command_viz_pub.publish(joy_twist_viz_msg)

    def joy_command(self):
        joy_twist_viz_msg = TwistStamped()
        joy_msg = Joy()
        joy_msg.axes = [0.0]*20
        joy_msg.axes[axis_body_roll] = self.twist_ang_msg.angular.x
        joy_msg.axes[axis_body_pitch] = self.twist_ang_msg.angular.y
        joy_msg.axes[axis_body_yaw  ] = self.twist_lin_ang_z_msg.angular.z
        joy_msg.axes[axis_body_y_off] = self.twist_lin_msg.linear.x
        joy_msg.axes[axis_body_x_off] = self.twist_lin_msg.linear.y
        joy_msg.axes[axis_body_z_off] = self.twist_lin_ang_z_msg.linear.z
        joy_msg.axes[axis_fi_x      ] = self.twist_lin_msg.linear.x
        joy_msg.axes[axis_fi_y      ] = self.twist_lin_msg.linear.y
        joy_msg.axes[axis_alpha     ] = self.twist_ang_msg.angular.x
        joy_msg.axes[axis_scale     ] = self.twist_ang_msg.angular.y

        joy_msg.buttons = [0]*17
        joy_msg.buttons[button_left_shift   ] = self.btn_left_shift_msg
        joy_msg.buttons[button_right_shift  ] = self.btn_right_shift_msg
        joy_msg.buttons[button_start        ] = self.btn_start_msg
        joy_msg.buttons[button_gait_switch  ] = self.btn_gait_switch_msg

        self.joy_command_pub.publish(joy_msg)

        joy_twist_viz_msg.header.frame_id = "base_link"
        joy_twist_viz_msg.header.stamp = self.get_clock().now().to_msg()
        joy_twist_viz_msg.twist.linear.x = self.twist_lin_msg.linear.x
        joy_twist_viz_msg.twist.linear.y = self.twist_lin_msg.linear.y
        joy_twist_viz_msg.twist.linear.z = self.twist_lin_msg.linear.z
        joy_twist_viz_msg.twist.angular.x = self.twist_lin_msg.angular.x
        joy_twist_viz_msg.twist.angular.y = self.twist_lin_msg.angular.y
        joy_twist_viz_msg.twist.angular.z = self.twist_lin_msg.angular.z
        
        self.joy_command_viz_pub.publish(joy_twist_viz_msg)
        
    def timer_cb(self):
        if self.joy_update:
            self.joy_command()
            self.joy_update = False

def main(args=None):
    rclpy.init(args=args)
    node = HexapodVelocityControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
