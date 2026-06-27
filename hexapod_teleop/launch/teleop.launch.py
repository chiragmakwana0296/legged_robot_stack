import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare parameters as launch arguments
    max_linear_velocity_arg = DeclareLaunchArgument(
        name='max_linear_velocity',
        default_value='0.2',
        description='Maximum linear velocity magnitude (m/s)'
    )

    max_angular_velocity_arg = DeclareLaunchArgument(
        name='max_angular_velocity',
        default_value='0.5',
        description='Maximum angular velocity (rad/s)'
    )

    max_alpha_arg = DeclareLaunchArgument(
        name='max_alpha',
        default_value='0.15',
        description='Maximum yaw rotation angle per step'
    )

    gait_type_arg = DeclareLaunchArgument(
        name='gait_type',
        default_value='2',
        description='Gait type (1 = RIPPLE, 2 = TRIPOD)'
    )

    watchdog_timeout_arg = DeclareLaunchArgument(
        name='watchdog_timeout',
        default_value='0.5',
        description='Time in seconds to wait for a cmd_vel message before stopping'
    )

    auto_stand_up_arg = DeclareLaunchArgument(
        name='auto_stand_up',
        default_value='true',
        description='Automatically stand up when a cmd_vel message is received'
    )

    # Teleop twist node
    teleop_twist_node = Node(
        package='hexapod_teleop',
        executable='teleop_twist',
        name='teleop_twist_node',
        output='screen',
        parameters=[{
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'max_alpha': LaunchConfiguration('max_alpha'),
            'gait_type': LaunchConfiguration('gait_type'),
            'watchdog_timeout': LaunchConfiguration('watchdog_timeout'),
            'auto_stand_up': LaunchConfiguration('auto_stand_up')
        }]
    )

    return LaunchDescription([
        max_linear_velocity_arg,
        max_angular_velocity_arg,
        max_alpha_arg,
        gait_type_arg,
        watchdog_timeout_arg,
        auto_stand_up_arg,
        teleop_twist_node
    ])
