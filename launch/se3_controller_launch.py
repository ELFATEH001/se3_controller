from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('se3_controller')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    controller_node = Node(
        package='se3_controller',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/mavros/local_position/odom', '/mavros/local_position/odom'),
            ('/mavros/setpoint_attitude/attitude', '/mavros/setpoint_attitude/attitude'),
        ]
    )

    return LaunchDescription([
        controller_node,
    ])