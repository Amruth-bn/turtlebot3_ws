#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_dir = os.path.join(get_package_share_directory('ecn_turtlebot3x'), 'launch')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/ecn_waffle_nav2.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_dir, '/ecn_waffle_localization.launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,}.items(),
        ),
    ])