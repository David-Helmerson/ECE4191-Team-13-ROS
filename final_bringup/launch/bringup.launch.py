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
            ],
            arguments=[('__log_level:=debug')]
        ),

        Node(
            package='serial_communication',
            executable='serial_sender',
            arguments=[('__log_level:=debug')]
        ),

        Node(
            package='serial_communication',
            executable='serial_reader',
            arguments=[('__log_level:=debug')]
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