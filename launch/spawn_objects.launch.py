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
    spawn_bed = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'bed.model')] , 
            '-entity', 'bed',
            '-x', '5.0', '-y', '-3.9','-Y', '3.1415926']
        )
    spawn_sofa = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'sofa.model')] , 
            '-entity', 'sofa',
            '-x', '-1.0', '-y', '-3.9','-Y', '1.57']
        )
    spawn_tea_table = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'tea_table.model')] , 
            '-entity', 'tea_table',
            '-x', '-2.1', '-y', '-2.2','-Y', '1.57']
        )
    spawn_bookshelft = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'bookshelft.model')] , 
            '-entity', 'bookshelft',
            '-x', '2.0', '-y', '-0.55','-Y', '-1.57']
        )

    spawn_kitchen_table = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'table.model')] , 
            '-entity', 'kitchen_table',
            '-x', '-3.5', '-y', '3.7','-Y', '1.57']
        )
    spawn_red_bottle = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'bottles', 'red_bottle.model')] , 
            '-entity', 'red_bottle',
            '-x', '-3.3', '-y', '3.55','-z', '2']
        )
    spawn_green_bottle = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'bottles', 'green_bottle.model')] , 
            '-entity', 'green_bottle',
            '-x', '-3.6', '-y', '3.55','-z', '2']
        )
    
    spawn_cupboard_0 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'cupboard.model')] , 
            '-entity', 'cupboard_0',
            '-x', '-2.0', '-y', '0.7' ,'-Y', '1.57']
        )
    spawn_cupboard_1 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'cupboard.model')] , 
            '-entity', 'cupboard_1',
            '-x', '-1.3', '-y', '3.7','-Y', '-1.57']
        )
    
    spawn_dinning_table_0 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'table.model')] , 
            '-entity', 'dinning_table_0',
            '-x', '1.5', '-y', '1.5','-Y', '1.57']
        )
    spawn_dinning_table_1 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'table.model')] , 
            '-entity', 'dinning_table_1',
            '-x', '1.5', '-y', '2.0','-Y', '1.57']
        )
    spawn_dinning_table_2 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'table.model')] , 
            '-entity', 'dinning_table_2',
            '-x', '2.7', '-y', '1.5','-Y', '1.57']
        )
    spawn_dinning_table_3 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'table.model')] , 
            '-entity', 'dinning_table_3',
            '-x', '2.7', '-y', '2.0','-Y', '1.57']
        )
    
    spawn_chair_0 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'chair.model')] , 
            '-entity', 'chair_0',
            '-x', '1.5', '-y', '1.2','-Y', '1.57']
        )
    spawn_chair_1 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'chair.model')] , 
            '-entity', 'chair_1',
            '-x', '1.5', '-y', '2.3','-Y', '-1.57']
        )
    spawn_chair_2 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'chair.model')] , 
            '-entity', 'chair_2',
            '-x', '2.7', '-y', '1.2','-Y', '1.57']
        )
    spawn_chair_3 = Node(
            package='gazebo_ros',executable='spawn_entity.py',name='spawn_entity',arguments=['-file', [os.path.join(get_package_share_directory('wpr_simulation2'), 'models', 'chair.model')] , 
            '-entity', 'chair_3',
            '-x', '2.7', '-y', '2.3','-Y', '-1.57']
        )

    ld = LaunchDescription()

    ld.add_action(TimerAction(period=1.0, actions=[spawn_bed,spawn_sofa,spawn_tea_table,spawn_bookshelft]))
    # ld.add_action(spawn_bed)
    # ld.add_action(spawn_sofa)
    # ld.add_action(spawn_tea_table)
    # ld.add_action(spawn_bookshelft)
    
    ld.add_action(TimerAction(period=2.0, actions=[spawn_kitchen_table,spawn_cupboard_0,spawn_cupboard_1]))
    # ld.add_action(spawn_kitchen_table)
    # ld.add_action(spawn_cupboard_0)
    # ld.add_action(spawn_cupboard_1)
    
    ld.add_action(TimerAction(period=3.0, actions=[spawn_dinning_table_0,spawn_dinning_table_1,spawn_dinning_table_2,spawn_dinning_table_3]))
    # ld.add_action(spawn_dinning_table_0)
    # ld.add_action(spawn_dinning_table_1)
    # ld.add_action(spawn_dinning_table_2)
    # ld.add_action(spawn_dinning_table_3)
    
    ld.add_action(TimerAction(period=4.0, actions=[spawn_chair_0,spawn_chair_1,spawn_chair_2,spawn_chair_3]))
    # ld.add_action(spawn_chair_0)
    # ld.add_action(spawn_chair_1)
    # ld.add_action(spawn_chair_2)
    # ld.add_action(spawn_chair_3)

    ld.add_action(TimerAction(period=5.0, actions=[spawn_red_bottle,spawn_green_bottle]))

    return ld
