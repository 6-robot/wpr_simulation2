import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    launch_file_dir = os.path.join(get_package_share_directory('wpr_simulation2'), 'launch')
    robocup_home_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robocup_home.launch.py')
        )
    )

    slam_params = {
        "use_sim_time": True,
        "base_frame": "base_footprint",
        "odom_frame": "odom",
        "map_frame": "map",
    }
    slam_cmd = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        parameters=[slam_params]
    )

    rviz_file = os.path.join(get_package_share_directory('wpr_simulation2'), 'rviz', 'slam.rviz')
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file]
    )

    keyboard_vel_cmd = Node(
        package="wpr_simulation2",
        executable="keyboard_vel_cmd",
        name="keyboard_vel_cmd",
        prefix='gnome-terminal --'
    )

    ld = LaunchDescription()
    ld.add_action(robocup_home_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(keyboard_vel_cmd)

    return ld