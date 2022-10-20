from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            parameters=[
                {"width": 640},
                {"height": 480},
                {"frequency": 5.0}
            ],
            arguments=['--ros-args', '--log-level', 'warn']
        ),

        Node(
            package='serial_communication',
            executable='serial_sender',
            arguments=['--ros-args', '--log-level', 'warn']
        ),

        Node(
            package='serial_communication',
            executable='serial_reader',
            arguments=['--ros-args', '--log-level', 'warn']
        ),

        Node(
            package='localization',
            executable='encoder_predictor',
        ),

        Node(
            package='perception',
            executable='yolo',
            parameters=[
                {"model_name": '500kb_targets.pt'}
            ]
        ),

        Node(
            package='state_control',
            executable='backup_handler',
        ),
    ])