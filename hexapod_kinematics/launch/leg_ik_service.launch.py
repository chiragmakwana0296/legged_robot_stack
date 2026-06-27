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
    
    leg_ik_service_node = Node(
        package='hexapod_kinematics',
        executable='leg_ik_service_node',
        name='hexapod_leg_kinematics',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    body_kinematics_node = Node(
        package='hexapod_body_kinematics',
        executable='hexapod_body_kinematics_node',
        name='hexapod_body_kinematics',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    joint_publisher_node = Node(
        package='hexapod_kinematics',
        executable='hexapod_joint_publisher_node',
        name='hexapod_joint_publisher_node',
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        leg_ik_service_node,
        body_kinematics_node,
        joint_publisher_node
    ])
