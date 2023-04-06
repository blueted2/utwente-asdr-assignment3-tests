from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jiwy_simulator',
            # namespace='jiwy_simulator',
            executable='jiwy_simulator',
            name='sim'
        ),
    ])