import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('leg_description')
    
    # Default model path
    model_path = os.path.join(pkg_dir, 'urdf', 'body.xacro')
    rviz_config_path = os.path.join(pkg_dir, 'launch', 'urdf.rviz')
    
    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=model_path,
        description='Absolute path to robot urdf file'
    )
    
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=rviz_config_path,
        description='Absolute path to rviz config file'
    )
    
    # Robot State Publisher Node
    # In ROS 2, Command substitution runs the xacro parser
    robot_description = Command(['xacro ', LaunchConfiguration('model')])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'ignore_timestamp': True}]
    )
    
    # Rviz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher_node,
        # rviz_node
    ])
