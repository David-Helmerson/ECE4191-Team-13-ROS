from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            namespace='image_collector',
            executable='marble_classifier',
            name='marble_classifier'
        ),

        Node(
            package='perception',
            namespace='image_collector',
            executable='image_snapshot',
            name='snapshot'
        ),
        
    ])