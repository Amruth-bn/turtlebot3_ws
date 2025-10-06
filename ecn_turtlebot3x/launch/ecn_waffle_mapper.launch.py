#!/usr/bin/env python3

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('turtlebot3_gazebo'),
        #             'launch',
        #             'turtlebot3_world.launch.py'
        #         ])
        #     ]),
        # ),
        # Node(
        #     package = 'ecn_turtlebot3x',
        #     executable = 'waffle_odometry',
        #     name = 'waffle_odometry',
        #     output = 'screen',
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('ecn_turtlebot3x'),
        #             'launch',
        #             'ecn_waffle_localization.launch.py'
        #         ])
        #     ])
        # ),
        # Launch slam_toolbox online_async_launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'true',
                'slam_params_file': PathJoinSubstitution([
                    FindPackageShare('ecn_turtlebot3x'),
                    'config',
                    'slam_toolbox.yaml'
                ])
            }.items()
        ),
        # Launch rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
    ])