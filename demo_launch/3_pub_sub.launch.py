from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    publisher_cmd = Node(
            package='wpr_simulation2',
            executable='3_publisher_node',
            name='publisher_node'
        )
    
    subscriber_cmd = Node(
            package='wpr_simulation2',
            executable='3_subscriber_node',
            name='subscriber_node'
        )

    ld = LaunchDescription()
    ld.add_action(publisher_cmd)
    ld.add_action(subscriber_cmd)
    return ld