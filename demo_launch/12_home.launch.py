import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')
    home_mani_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robocup_home_mani.launch.py')
        )
    )

    map_file = os.path.join(
        get_package_share_directory('wpr_simulation2'),
        'maps',
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
            'use_sim_time': "True",
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

    rviz_file = os.path.join(get_package_share_directory('wpr_simulation2'), 'rviz', 'fetch.rviz')

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    ld = LaunchDescription()
    ld.add_action(home_mani_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(wp_edit_cmd)
    ld.add_action(wp_navi_server_cmd)
    ld.add_action(objects_publisher_cmd)
    ld.add_action(grab_object_cmd)
    ld.add_action(rviz_cmd)
    
    return ld