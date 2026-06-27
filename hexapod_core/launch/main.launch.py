import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leg_desc = get_package_share_directory('leg_description')
    
    # Default model path (URDF/Xacro)
    model_path = os.path.join(pkg_leg_desc, 'urdf', 'body.xacro')
    
    # Launch configuration variables
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=model_path,
        description='Absolute path to robot urdf file'
    )
    
    # Generate robot_description parameter using xacro
    robot_description = Command(['xacro ', LaunchConfiguration('model')])
    
    # 1. leg_ik_service_node (hexapod_kinematics)
    leg_ik_service_node = Node(
        package='hexapod_kinematics',
        executable='leg_ik_service_node',
        name='hexapod_leg_kinematics',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 2. hexapod_body_kinematics_node (hexapod_body_kinematics)
    body_kinematics_node = Node(
        package='hexapod_body_kinematics',
        executable='hexapod_body_kinematics_node',
        name='hexapod_body_kinematics',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 3. hexapod_joint_publisher_node (hexapod_kinematics)
    joint_publisher_node = Node(
        package='hexapod_kinematics',
        executable='hexapod_joint_publisher_node',
        name='hexapod_joint_publisher_node',
        output='screen'
    )
    
    # 4. gait_kinematics_node (hexapod_gait_kinematics)
    gait_kinematics_node = Node(
        package='hexapod_gait_kinematics',
        executable='gait_kinematics_node',
        name='gait_kinematics_node',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 5. velocity_control_node (hexapod_core)
    velocity_control_node = Node(
        package='hexapod_core',
        executable='velocity_control.py',
        name='velocity_control_node',
        output='screen'
    )
    
    # 6. joint_state_to_trajectory_node (hexapod_core)
    joint_state_to_trajectory_node = Node(
        package='hexapod_core',
        executable='joint_state_to_trajectory.py',
        name='joint_state_to_trajectory_node',
        output='screen'
    )
    
    # 7. gait_teleop_node (hexapod_gait_kinematics)
    gait_teleop_node = Node(
        package='hexapod_gait_kinematics',
        executable='gait_teleop_node',
        name='gait_teleop_node',
        output='screen'
    )
    
    # 8. joy_node (joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        leg_ik_service_node,
        body_kinematics_node,
        joint_publisher_node,
        gait_kinematics_node,
        velocity_control_node,
        joint_state_to_trajectory_node,
        gait_teleop_node,
        joy_node
    ])
