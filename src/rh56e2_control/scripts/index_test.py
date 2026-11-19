#!/usr/bin/env python3
"""Hold other joints at their initial positions and command index 0 -> 1 rad."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class IndexTest(Node):
    def __init__(self):
        super().__init__('index_test')
        self.names = [
            'thumb_joint1', 'thumb_joint2', 'index_joint',
            'middle_joint', 'ring_joint', 'pinky_joint',
        ]
        self.command = None
        self.position = None
        self.target = 0.0
        self.zero_since = None
        self.started = time.monotonic()
        self.last_log = 0.0
        self.feedback_at = None

        self.pub = self.create_publisher(
            Float64MultiArray,
            '/hand_joint_controller/target_joint_position', 10,
        )
        self.sub = self.create_subscription(
            JointState, '/joint_states',
            self.receive_state, qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.1, self.step)

    def receive_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions and math.isfinite(positions[name])
                   for name in self.names):
            return

        self.position = positions['index_joint']
        self.feedback_at = time.monotonic()
        if self.command is None:
            self.command = [positions[name] for name in self.names]

    def step(self):
        now = time.monotonic()
        if now - self.started > 30.0:
            raise RuntimeError('30초 테스트 종료')
        if self.command is None or self.pub.get_subscription_count() == 0:
            return
        if now - self.feedback_at > 1.0:
            raise RuntimeError('관절 피드백 수신 중단')

        # Hold index at zero for one second, then step its target to 1 rad.
        if self.target == 0.0:
            if abs(self.position) <= 0.01:
                if self.zero_since is None:
                    self.zero_since = now
                elif now - self.zero_since >= 1.0:
                    self.target = 1.0
                    self.get_logger().info('검지 목표: 0 → 1.0 rad')
            else:
                self.zero_since = None

        self.command[2] = self.target
        msg = Float64MultiArray()
        msg.data = self.command
        self.pub.publish(msg)

        if now - self.last_log >= 0.5:
            self.last_log = now
            self.get_logger().info(
                f'index target={self.target:.3f} actual={self.position:.3f} rad')


def main():
    rclpy.init()
    node = IndexTest()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as error:
        print(error)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
