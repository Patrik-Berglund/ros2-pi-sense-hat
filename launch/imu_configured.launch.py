#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('ros2_pi_sense_hat'),
            'config',
            'imu_params.yaml'
        ]),
        description='Path to IMU configuration file'
    )
    
    # IMU node with parameters
    imu_node = Node(
        package='ros2_pi_sense_hat',
        executable='imu_node',
        name='imu_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        imu_node
    ])
