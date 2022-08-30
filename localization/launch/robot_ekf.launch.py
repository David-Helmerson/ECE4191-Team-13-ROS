from http.server import executable
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='localization').find('localization')
    config_path = os.path.join(pkg_share, 'config/ekf.yaml') 
    print(config_path)

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_path, 
        {'use_sim_time': False}])


    ld = LaunchDescription()
    ld.add_action(start_robot_localization_cmd)
    return ld