# legged_robot_stack
Control software stack for legged robots


roslaunch leg_description display.launch 
roslaunch hexapod_kinematics leg_ik_service.launch
rosrun hexapod_gait_kinematics gait_kinematics_node
rosrun hexapod_core velocity_control.py
rosrun hexapod_gait_kinematics gait_teleop_node 