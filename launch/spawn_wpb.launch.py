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
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the urdf file
    urdf_path = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'models',
        'wpb_home.model'
    )

    # Process the URDF file
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
        doc = xacro.parse(robot_desc)
        xacro.process_doc(doc)
        robot_description = doc.toxml()
    
    robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }],
        )

    # Launch configuration variables specific to simulation
    pose_x = LaunchConfiguration('pose_x', default='0.0')
    pose_y = LaunchConfiguration('pose_y', default='0.0')
    pose_theta = LaunchConfiguration('pose_theta', default='0.0')

    # Declare the launch arguments
    # declare_x_position_cmd = DeclareLaunchArgument(
    #     'x_pose', default_value='0.0',
    #     description='Specify namespace of the robot')

    # declare_y_position_cmd = DeclareLaunchArgument(
    #     'y_pose', default_value='0.0',
    #     description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', "wpb_home",
            '-x', pose_x,
            '-y', pose_y,
            '-Y', pose_theta
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    # ld.add_action(declare_x_position_cmd)
    # ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
