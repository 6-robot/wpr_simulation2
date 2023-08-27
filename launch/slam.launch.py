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
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robocup_home.launch.py')
        )
    )

    slam_cmd = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[{
            "use_sim_time": True,
            "base_frame": "base_footprint",
            "odom_frame": "odom",
            "map_frame": "map"
        }]
    )

    rviz_cmd = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', [os.path.join(get_package_share_directory('wpr_simulation2'), 'rviz', 'slam.rviz')]]
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gazebo_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)

    return ld
