from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_communication',
            namespace='prelim',
            executable='serial_reader',
            name='serial_reader'
        ),
        Node(
            package='serial_communication',
            namespace='prelim',
            executable='serial_sender',
            name='serial_sender'
        ),
        Node(
            package='localization_python',
            namespace='prelim',
            executable='encoder_predictor',
            name='encoder_predictor'
        ),
        Node(
            package='navigation',
            namespace='prelim',
            executable='waypoint_manager',
            name='waypoint_manager'
        ),
        Node(
            package='navigation',
            namespace='prelim',
            executable='tentacle_planner',
            name='tentacle_planner'
        ),
    ])