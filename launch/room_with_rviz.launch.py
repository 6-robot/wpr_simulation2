#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wpr_simulation2')
    lidar_room_sim_launch = os.path.join(pkg_share, 'launch', 'lidar_room.launch.py')
    rviz_config = os.path.join(pkg_share, 'rviz', 'lidar_3d.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    headless = LaunchConfiguration('headless', default='false')
    robot_x = LaunchConfiguration('robot_x', default='0.0')
    robot_y = LaunchConfiguration('robot_y', default='0.0')
    robot_z = LaunchConfiguration('robot_z', default='0.0')

    # 1. 启动仿真环境（Gazebo + 机器人状态发布 + 生成机器人）
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_room_sim_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'headless': headless,
            'robot_x': robot_x,
            'robot_y': robot_y,
            'robot_z': robot_z,
        }.items(),
    )

    # 2. 延迟 3 秒启动 RViz2
    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(rviz),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='使用仿真时钟'),
        DeclareLaunchArgument('rviz', default_value='true', description='是否启动 RViz2'),
        DeclareLaunchArgument('headless', default_value='false', description='是否以无 GUI 模式运行 Gazebo'),
        DeclareLaunchArgument('robot_x', default_value='0.0', description='机器人出生点 X (m)'),
        DeclareLaunchArgument('robot_y', default_value='0.0', description='机器人出生点 Y (m)'),
        DeclareLaunchArgument('robot_z', default_value='0.0', description='机器人出生点 Z (m)'),
        included_launch,
        rviz_node,
    ])
