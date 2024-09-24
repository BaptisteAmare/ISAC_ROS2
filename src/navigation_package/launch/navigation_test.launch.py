from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation_package',
            executable='waypoint_manager_node',
            name='waypoint_manager_node'
        ),
        Node(
            package='navigation_package',
            executable='navigation_calculator_node',
            name='navigation_calculator_node'
        )
    ])
