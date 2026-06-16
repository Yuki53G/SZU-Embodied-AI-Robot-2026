#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('originbot_nav2_competition')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = '/root/ros2_ws/maps/originbot_competition_map.yaml'
    default_params = os.path.join(pkg_dir, 'config', 'originbot_nav2_competition.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_mission = LaunchConfiguration('use_mission')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )

    mission_node = Node(
        package='originbot_nav2_competition',
        executable='nav2_mission_node',
        name='nav2_mission_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # 说明：这里默认只启动 Nav2，不自动启动 mission。
    # 等 map->base_link 和单点 goal 测试成功后，再单独 ros2 run nav2_mission_node。
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_mission', default_value='false'),
        nav2,
    ])
