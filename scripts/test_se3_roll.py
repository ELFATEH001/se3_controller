import rclpy
from rclpy.node import Node
from mavros_msgs.msg import AttitudeTarget
import math
import time


class SE3RollTest(Node):
    def __init__(self):
        super().__init__('se3_roll_test')
        self.publisher = self.create_publisher(
            AttitudeTarget,
            '/mavros/setpoint_attitude/attitude',
            10
        )
        # Publish at 50 Hz
        self.timer = self.create_timer(0.02, self.publish_setpoint)
        self.start_time = time.time()
        self.phase = 'hover'   # hover → roll → recover
        self.get_logger().info('SE3 Roll Test started...')

    def publish_setpoint(self):
        elapsed = time.time() - self.start_time
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        # type_mask = 7 → use orientation + thrust, ignore body rates
        msg.type_mask = 7
        msg.thrust = 0.567  # same hover thrust your SE3 computed

        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0

        if elapsed < 3.0:
            # Phase 1: Hover level for 3 seconds (baseline)
            self.phase = 'hover'
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.6976
            msg.orientation.w = 0.7164

        elif elapsed < 6.0:
            # Phase 2: Command 10° roll for 3 seconds
            self.phase = 'roll'
            roll_rad = math.radians(10)
            msg.orientation.x = math.sin(roll_rad / 2)   # ~0.0872
            msg.orientation.y = 0.0
            msg.orientation.z = 0.6976
            msg.orientation.w = math.cos(roll_rad / 2)   # ~0.9962 combined with yaw

        else:
            # Phase 3: Recover to hover
            self.phase = 'recover'
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = 0.6976
            msg.orientation.w = 0.7164

        self.publisher.publish(msg)
        self.get_logger().info(
            f'[{self.phase.upper()}] t={elapsed:.1f}s | '
            f'roll_x={msg.orientation.x:.4f} | thrust={msg.thrust:.3f}',
            throttle_duration_sec=0.5  # log every 0.5s to avoid spam
        )


def main():
    rclpy.init()
    node = SE3RollTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Test stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()