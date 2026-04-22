from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import datetime


def generate_launch_description():

    pkg_share = get_package_share_directory('se3_controller')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Timestamped bag name — evaluated at launch time
    bag_name = 'se3_debug_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    # -------------------------------------------------------
    # SE(3) controller node — loads gains from params.yaml
    # -------------------------------------------------------
    controller_node = Node(
        package='se3_controller',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[params_file],
    )

    # -------------------------------------------------------
    # ros2 bag record — all topics needed for SE(3) debug
    # -------------------------------------------------------
    bag_recorder = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', bag_name,

            # --- State feedback ---
            '/mavros/local_position/odom',           # position, velocity, orientation
            '/mavros/local_position/pose',           # pose only
            '/mavros/imu/data',                      # raw IMU (angular rates, accel)

            # --- Controller output ---
            '/mavros/setpoint_raw/attitude',         # what SE(3) commands

            # --- MAVROS loopback (confirms FCU received it) ---
            '/mavros/setpoint_raw/target_attitude',  # echoed back from FCU

            # --- ArduPilot inner loop state ---
            '/mavros/state',                         # mode, armed status
            '/mavros/rc/in',                         # RC input (emergency switch)

            # --- SE(3) internals debug topic ---
            '/se3/debug',                            # position error, thrust, eR, etc.
        ],
        output='screen'
    )

    return LaunchDescription([
        controller_node,
        bag_recorder,
    ])