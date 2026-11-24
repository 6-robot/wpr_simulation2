#!/usr/bin/env python3
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wpr_simulation2')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_share, 'worlds', 'scan_room.world')
    model_file = os.path.join(pkg_share, 'models', 'wpb_home_3d_lidar.model')

    # 解析机器人模型描述
    with open(model_file, 'r', encoding='utf-8') as f:
        robot_desc = f.read()
        doc = xacro.parse(robot_desc)
        xacro.process_doc(doc)
        robot_description = doc.toxml()

    # 启动参数定义
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='使用 Gazebo 仿真时钟 (/clock)',
    )
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='是否以无 GUI 模式运行 Gazebo',
    )
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='0.0', description='机器人出生点 X (m)')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='0.0', description='机器人出生点 Y (m)')
    robot_z_arg = DeclareLaunchArgument('robot_z', default_value='0.0', description='机器人出生点 Z (m)')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_z = LaunchConfiguration('robot_z')

    gui_enabled = PythonExpression(["'false' if '", headless, "' == 'true' else 'true'"])

    # 1. 启动 Gazebo Classic 仿真环境
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'gui': gui_enabled,
            'verbose': 'false',
        }.items(),
    )

    # 2. 发布机器人状态与 TF 变换
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # 3. 延迟 2 秒在 Gazebo 中生成 wpb_home_3d_lidar 机器人
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_wpb_home',
                output='screen',
                arguments=[
                    '-entity', 'wpb_home',
                    '-topic', 'robot_description',
                    '-x', robot_x,
                    '-y', robot_y,
                    '-z', robot_z,
                ],
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        headless_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        gazebo_sim,
        robot_state_publisher,
        spawn_robot,
    ])
