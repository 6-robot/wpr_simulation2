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
from launch.actions import TimerAction

def generate_launch_description():
    spawn_man_1 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'man_1.model')] , 
            '-entity', 'man_1',
            '-x', '4.0', '-y', '1.0','-Y', '3.1415926']
        )

    spawn_man_2 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'man_2.model')] , 
            '-entity', 'man_2',
            '-x', '4.0', '-y', '1.0','-Y', '3.1415926']
        )
    
    spawn_woman = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'woman.model')] , 
            '-entity', 'woman',
            '-x', '4.0', '-y', '1.0','-Y', '3.1415926']
        )
    ld = LaunchDescription()

    ld.add_action(TimerAction(period=7.0, actions=[spawn_man_2]))

    return ld
