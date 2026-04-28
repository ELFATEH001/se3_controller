from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    return LaunchDescription([

        # # 1. Gazebo world with ArUco
        # ExecuteProcess(
        #     cmd=['gz', 'sim',
        #          '/home/drone/dc_ws/src/se3_controller/worlds/aruco_landing.world',
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
                executable='controller_node_visual',
                output='screen'
            ),
        ]),
    ])