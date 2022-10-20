from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            parameters=[
                {"width": 640},
                {"height": 480},
                {"frequency": 5.0}
            ]
        ),

        Node(
            package='serial_communication',
            executable='serial_sender',
        ),

        Node(
            package='serial_communication',
            executable='serial_reader',
        ),

        Node(
            package='localization',
            executable='encoder_predictor',
        ),

        Node(
            package='perception',
            executable='yolo',
        ),

        Node(
            package='state_control',
            executable='backup_handler',
        ),
    ])