from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    logger = launch.substitutions.LaunchConfiguration("log_level")

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            "log_level",
            default_value=["debug"],
            description="Logging level",
        ),

        Node(
            package='image_tools',
            executable='cam2image',
            parameters=[
                {"width": 640},
                {"height": 480},
                {"frequency": 5.0}
            ],
            arguments=['--ros-args', '--log-level', logger]
        ),

        Node(
            package='serial_communication',
            executable='serial_sender',
            arguments=['--ros-args', '--log-level', logger]
        ),

        Node(
            package='serial_communication',
            executable='serial_reader',
            arguments=['--ros-args', '--log-level', logger]
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