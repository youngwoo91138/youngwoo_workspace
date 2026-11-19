#!/usr/bin/env python3
"""Inspect all tactile pads without publishing any robot commands."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


class TactileMonitor(Node):
    def __init__(self):
        super().__init__('tactile_monitor')
        self.samples = {}
        self.subscriptions_ = []
        self.fingers = ('thumb', 'index', 'middle', 'ring', 'pinky')
        for finger in self.fingers:
            self.samples[finger] = {
                'data': None, 'last': None, 'count': 0, 'bad': 0,
                'error': '', 'previous_count': 0,
            }
            self.subscriptions_.append(self.create_subscription(
                Float64MultiArray, f'/hand/tactile/{finger}_pad',
                lambda msg, f=finger: self.receive(msg, f), qos_profile_sensor_data))
        self.last_report = time.monotonic()
        self.timer = self.create_timer(1.0, self.report)
        self.get_logger().info(
            'Tactile-only monitor: no motion commands. Press one sensor surface at a time; '
            'check sum/max/nonzero and matching finger response. CoP has no pressure threshold here.')

    def receive(self, msg, finger):
        state = self.samples[finger]
        state['count'] += 1
        state['last'] = time.monotonic()
        state['data'] = None
        expected = 96 if finger == 'thumb' else 80
        if len(msg.data) != expected:
            state['bad'] += 1
            state['error'] = f'length={len(msg.data)}, expected={expected}'
        elif any(not math.isfinite(v) or v < 0 or v > 65535 for v in msg.data):
            state['bad'] += 1
            state['error'] = 'invalid taxel value'
        else:
            state['data'] = list(msg.data)
            state['error'] = ''

    def report(self):
        now = time.monotonic()
        interval = now - self.last_report
        self.last_report = now
        for finger, state in self.samples.items():
            topic = f'/hand/tactile/{finger}_pad'
            rate = (state['count'] - state['previous_count']) / interval
            state['previous_count'] = state['count']
            prefix = (f'{finger}: publishers={self.count_publishers(topic)} '
                      f'rx={state["count"]} rate={rate:.1f}Hz bad={state["bad"]}')
            if state['last'] is None:
                self.get_logger().info(f'{prefix} NO DATA')
                continue
            age = now - state['last']
            prefix += f' age={age:.2f}s'
            if age >= 1.5:
                prefix += ' STALE'
            data = state['data']
            if data is None:
                self.get_logger().info(f'{prefix} INVALID: {state["error"]}')
                continue
            total = sum(data)
            cop = 'undefined'
            if total > 0:
                x = sum(v * (i % 8) for i, v in enumerate(data)) / total
                y = sum(v * (i // 8) for i, v in enumerate(data)) / total
                cop = f'({x:.2f},{y:.2f})'
            self.get_logger().info(
                f'{prefix} sum={total:.1f} min={min(data):.1f} max={max(data):.1f} '
                f'nonzero={sum(v > 0 for v in data)}/{len(data)} raw_CoP={cop}')


def main():
    rclpy.init()
    node = TactileMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
