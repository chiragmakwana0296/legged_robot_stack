#!/usr/bin/env python3

from turtle import stamp
import rospy
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


class HexapodVelocityControl():
    def __init__(self):
        rospy.init_node("hexapod_velocity_control")
        # self.gait_command_pub = rospy.Publisher("/teleop/gait_control", GaitCommand, queue_size=1)
        self.joy_command_pub = rospy.Publisher("/joy_out", Joy, queue_size=1)
        self.joy_command_viz_pub = rospy.Publisher("/joy_twist_viz", TwistStamped, queue_size=1)
        
        rospy.Subscriber("/cmd_vel/angular", Twist, self.cmd_vel_ang_cb)
        rospy.Subscriber("/cmd_vel/linear", Twist, self.cmd_vel_lin_cb)
        rospy.Subscriber("/cmd_vel/lin_ang_z", Twist, self.cmd_vel_lin_ang_z_cb)
       
        rospy.Subscriber("/btn/gait_switch", Bool, self.gait_switch_cb)
        rospy.Subscriber("/btn/left_shift", Bool, self.left_shift_cb)
        rospy.Subscriber("/btn/right_shift", Bool, self.right_shift_cb)
        rospy.Subscriber("/btn/start", Bool, self.start_cb)

        rospy.Subscriber("/joy", Joy, self.joy_cb)
        
        self.btn_gait_switch_msg = int()
        self.btn_left_shift_msg = int()
        self.btn_right_shift_msg = int()
        self.btn_start_msg = int()

        self.twist_ang_msg = Twist()
        self.twist_lin_msg = Twist()
        self.twist_lin_ang_z_msg = Twist()

        self.joy_update = False


################### Button Cb ######################################
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

    def gait_cmd_vel_cb(self, msg):
        self.btn__msg = int(msg.data)
        self.joy_update = True

################### Twist Cb ######################################

    def cmd_vel_ang_cb(self, msg):
        self.twist_ang_msg = msg
        self.joy_update = True

    def cmd_vel_lin_cb(self, msg):
        self.twist_lin_msg = msg
        self.joy_update = True
    
    def cmd_vel_lin_ang_z_cb(self, msg):
        self.twist_lin_ang_z_msg = msg
        self.joy_update = True
    
    def joy_cb(self,msg):
        joy_twist_viz_msg = TwistStamped()
        joy_msg = Joy()
        joy_msg.axes = [0]*20
        joy_msg.axes[axis_body_roll] =  msg.axes[0]  #self.twist_ang_msg.angular.x
        joy_msg.axes[axis_body_pitch] = msg.axes[1]   #self.twist_ang_msg.angular.y
        joy_msg.axes[axis_body_yaw  ] = msg.axes[4]  #self.twist_lin_ang_z_msg.angular.z

        joy_msg.axes[axis_body_x_off] = msg.axes[2]   #self.twist_lin_msg.linear.x
        joy_msg.axes[axis_body_y_off] = msg.axes[3]   #self.twist_lin_msg.linear.y
        joy_msg.axes[axis_body_z_off] = msg.axes[5]   #self.twist_lin_ang_z_msg.linear.z

        joy_msg.axes[axis_fi_x      ] = msg.axes[3]   #self.twist_lin_msg.linear.x
        joy_msg.axes[axis_fi_y      ] = msg.axes[2]   #self.twist_lin_msg.linear.y

        joy_msg.axes[axis_alpha     ] = msg.axes[0]   #self.twist_ang_msg.angular.x
        joy_msg.axes[axis_scale     ] = msg.axes[1]   #self.twist_ang_msg.angular.y

        joy_msg.buttons = [0]*17
        joy_msg.buttons[button_left_shift   ] = msg.buttons[4]      #self.btn_left_shift_msg
        joy_msg.buttons[button_right_shift  ] = msg.buttons[5]      #self.btn_right_shift_msg
        joy_msg.buttons[button_right_shift_2] = msg.buttons[7]
        joy_msg.buttons[button_start        ] = msg.buttons[9]      #self.btn_start_msg
        joy_msg.buttons[button_gait_switch  ] =  msg.buttons[8]     #self.btn_gait_switch_msg

        self.joy_command_pub.publish(joy_msg)

        joy_twist_viz_msg.header.frame_id = "base_link"
        joy_twist_viz_msg.header.stamp = rospy.Time.now()
        joy_twist_viz_msg.twist.linear.x = self.twist_lin_msg.linear.x
        joy_twist_viz_msg.twist.linear.y = self.twist_lin_msg.linear.y
        joy_twist_viz_msg.twist.linear.z = self.twist_lin_msg.linear.z
        joy_twist_viz_msg.twist.angular.x = self.twist_lin_msg.angular.x
        joy_twist_viz_msg.twist.angular.y = self.twist_lin_msg.angular.y
        joy_twist_viz_msg.twist.angular.z = self.twist_lin_msg.angular.z
        
        self.joy_command_viz_pub.publish(joy_twist_viz_msg)
#########################################################

    def gait_command(self, msg):
        gait_command_msg = GaitCommand()
        a = math.pow(self.gait_cmd_vel_msg.linear.x, 2)
        b = math.pow(self.gait_cmd_vel_msg.linear.y, 2)
        gait_command_msg.cmd = 2
        gait_command_msg.fi = math.atan2(self.gait_cmd_vel_msg.linear.x, self.gait_cmd_vel_msg.linear.y)
        gait_command_msg.scale = math.pow(a+b, 0.5)
        if gait_command_msg.scale > 1:
            gait_command_msg.scale = 1.0

        gait_command_msg.alpha = 0.0

        print(gait_command_msg)
        self.gait_command_pub.publish(gait_command_msg)

    def joy_command(self):
        # note on plain values:
        # buttons are either 0 or 1
        # button axes go from 0 to -1
        # stick axes go from 0 to +/-1
        joy_twist_viz_msg = TwistStamped()
        joy_msg = Joy()
        joy_msg.axes = [0]*20
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
        # joy_msg.buttons[button_right_shift_2] = 
        joy_msg.buttons[button_start        ] = self.btn_start_msg
        joy_msg.buttons[button_gait_switch  ] = self.btn_gait_switch_msg

        self.joy_command_pub.publish(joy_msg)

        joy_twist_viz_msg.header.frame_id = "base_link"
        joy_twist_viz_msg.header.stamp = rospy.Time.now()
        joy_twist_viz_msg.twist.linear.x = self.twist_lin_msg.linear.x
        joy_twist_viz_msg.twist.linear.y = self.twist_lin_msg.linear.y
        joy_twist_viz_msg.twist.linear.z = self.twist_lin_msg.linear.z
        joy_twist_viz_msg.twist.angular.x = self.twist_lin_msg.angular.x
        joy_twist_viz_msg.twist.angular.y = self.twist_lin_msg.angular.y
        joy_twist_viz_msg.twist.angular.z = self.twist_lin_msg.angular.z
        
        self.joy_command_viz_pub.publish(joy_twist_viz_msg)
        

    def main(self):
        while not rospy.is_shutdown():
            if self.joy_update:
                self.joy_command()
                self.joy_update = False
            
            



if __name__ == "__main__":
    node = HexapodVelocityControl()
    node.main()
