import os
from datetime import datetime
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    pkg_share = get_package_share_directory('se3_controller')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Timestamp captured at launch time, not import time
    bag_name = '/home/aimane/dc_ws/bags/flight_' + datetime.now().strftime('%Y%m%d_%H%M%S')

    return LaunchDescription([

        # # 1. Gazebo world with ArUco
        # ExecuteProcess(
        #     cmd=['gz', 'sim',
        #          '/home/aimane/dc_ws/src/se3_controller/worlds/aruco_landing.world',
        #          '-v4', '-r'],
        #     output='screen'
        # ),

        # 2. Camera bridge (wait 3s for Gazebo to start)
        TimerAction(period=3.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                     '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                     '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                output='screen'
            ),
        ]),

        # 3. ArUco detector (wait 4s)
        TimerAction(period=4.0, actions=[
            Node(
                package='ros2_aruco',
                executable='aruco_node',
                parameters=[{
                    'marker_size': 0.40,
                    'aruco_dictionary_id': 'DICT_4X4_1000',
                    'image_topic': '/camera/image_raw',
                    'camera_info_topic': '/camera/camera_info',
                }],
                output='screen'
            ),
        ]),

        # 4. Mode manager (wait 5s — needs MAVROS up)
        TimerAction(period=5.0, actions=[
            Node(
                package='drone_mode_manager',
                executable='mode_manager_node',
                output='screen'
            ),
        ]),

        # 5. Visual servo node (wait 5s)
        TimerAction(period=5.0, actions=[
            Node(
                package='visual_servo_node',
                executable='visual_servo_node',
                output='screen'
            ),
        ]),

        # 6. SE3 controller (wait 5s)
        TimerAction(period=5.0, actions=[
            Node(
                package='se3_controller',
                executable='controller_node',
                name='controller_node',
                output='screen',
                parameters=[params_file],
            ),
        ]),

        # 7. Bag
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', bag_name,

                    # --- State feedback ---
                    '/mavros/local_position/odom',
                    '/mavros/local_position/pose',
                    '/mavros/imu/data',

                    # --- Controller I/O ---
                    '/mavros/setpoint_raw/attitude',
                    '/mavros/setpoint_raw/target_attitude',  # FCU loopback

                    # --- MAVROS status ---
                    '/mavros/state',
                    '/mavros/rc/in',

                    # --- SE(3) internals ---
                    '/se3/debug',

                    # --- Visual servo pipeline ---
                    '/visual_servo/velocity_setpoint',
                    '/visual_servo/enable',
                    '/visual_servo/debug',

                    # --- ArUco ---
                    '/aruco/markers',
                    '/aruco/debug_image',
                ],
                output='screen',
            ),
        ]),          # ← was missing: closes actions=[ and TimerAction(

    ])