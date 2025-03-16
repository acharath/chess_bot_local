from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_demo',  # Replace with your actual package name
            executable='internet',
            output='screen'
        )
    ])
