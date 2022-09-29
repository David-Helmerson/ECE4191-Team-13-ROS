from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            namespace='dp_test',
            executable='dp_test',
            name='dp_test'
        ),

        Node(
            package='perception',
            namespace='dp_test',
            executable='depth_perception',
            name='depth_perception',
            remappings=[
                ('image', 'test_image'),
                ('pose_est', 'test_pose'),
                ('encoder_vel', 'test_vel'),
            ]
        ),

        Node(
            package='image_tools',
            namespace='dp_test',
            executable='cam2image',
            name='camera'
        ),

        Node(
            package='tf2_ros',
            namespace='dp_test',
            executable='static_transform_publisher',
            name='tf_hack',
            arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        ),
        
    ])