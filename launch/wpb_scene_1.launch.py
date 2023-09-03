#!/usr/bin/env python3
#
# Copyright 2023 6-robot.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Zhang Wanjie

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    launch_file_dir = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')

    home_mani_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robocup_home_mani.launch.py')
        )
    )

    # 导航
    behavior_tree = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'config',
        'behavior_tree.xml'
    )

    map_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'map',
        'map.yaml'
    )
    
    nav_param_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'config',
        'nav2_params.yaml'
    )

    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 
        'launch'
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': nav_param_file}.items(),
    )

    wp_edit_cmd = Node(
            package='wp_map_tools',
            executable='wp_edit_node',
            name='wp_edit_node'
        )
    
    wp_navi_server_cmd = Node(
            package='wp_map_tools',
            executable='wp_navi_server',
            name='wp_navi_server'
        )

    # 抓取
    objects_publisher_cmd = Node(
            package='wpr_simulation2',
            executable='objects_publisher',
            name='objects_publisher',
            parameters=[
                {"auto_start": False}
            ]
        )

    grab_object_cmd = Node(
            package='wpr_simulation2',
            executable='grab_object_sim',
            name='grab_object_sim'
        )

    rviz_cmd = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(get_package_share_directory('wpr_simulation2'), 'rviz', 'fetch.rviz')]]
        )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(home_mani_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)

    ld.add_action(objects_publisher_cmd)
    ld.add_action(grab_object_cmd)
    
    ld.add_action(rviz_cmd)

    return ld
