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
    launch_file_dir = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'worlds',
        'wpb_simple.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_wpb.launch.py')
        )
    )

    spawn_orange_ball = Node(
            package='gazebo_ros',
            namespace='',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'balls', 'orange_ball.model')] , 
            '-entity', 'orange_ball',
            '-x', '2.0',
            '-y', '0.0',
            '-Y', '0']
        )

    spawn_red_ball = Node(
            package='gazebo_ros',
            namespace='',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'balls', 'red_ball.model')] , 
            '-entity', 'red_ball',
            '-x', '2.0',
            '-y', '0.5',
            '-Y', '0']
        )
    
    spawn_green_ball = Node(
            package='gazebo_ros',
            namespace='',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'balls', 'green_ball.model')] , 
            '-entity', 'green_ball',
            '-x', '2.0',
            '-y', '-0.5',
            '-Y', '0']
        )

    spawn_blue_ball = Node(
            package='gazebo_ros',
            namespace='',
            executable='spawn_entity.py',
            name='spawn_entity',
            arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'balls', 'blue_ball.model')] , 
            '-entity', 'blue_ball',
            '-x', '3.0',
            '-y', '0.0',
            '-Y', '0']
        )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ld.add_action(spawn_robot_cmd)
    ld.add_action(spawn_orange_ball)
    ld.add_action(spawn_red_ball)
    ld.add_action(spawn_green_ball)
    ld.add_action(spawn_blue_ball)

    return ld
