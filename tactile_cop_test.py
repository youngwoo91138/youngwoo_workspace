
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


class TactileCoPTest(Node):

    def __init__(self):
        super().__init__('tactile_cop_test')

        self.rows = 10
        self.cols = 8
        self.n = 80

        self.calibration_samples = 40
        self.min_sum = 2000.0
        self.slip_threshold = 0.8
        self.confirm_frames = 3

        self.baseline_samples = []
        self.baseline = None

        self.reference = None
        self.stable_cops = deque(maxlen=5)

        self.moving_frames = 0
        self.alerted = False
        self.last_print = 0.0

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/hand/tactile/index_pad',
            self.callback,
            qos_profile_sensor_data
        )

        print("=== Tactile CoP Test ===", flush=True)
        print("처음 40프레임 동안 센서를 누르지 마세요.",
              flush=True)

    def callback(self, msg):

        data = list(msg.data)

        if len(data) != self.n:
            return

        if not all(
            math.isfinite(v) and 0 <= v <= 65535
            for v in data
        ):
            return

        # STEP 1: 센서 초기 기준값 보정
        if self.baseline is None:
            self.baseline_samples.append(data)

            if len(self.baseline_samples) >= self.calibration_samples:
                self.baseline = [
                    sum(s[i] for s in self.baseline_samples)
                    / len(self.baseline_samples)
                    for i in range(self.n)
                ]

                print("\n[ZERO COMPLETE]", flush=True)
                print("검지 패드를 누르고 가만히 유지하세요.",
                      flush=True)

            return

        # STEP 2: 초기 기준값 제거
        pressure = [
            max(0.0, data[i] - self.baseline[i])
            for i in range(self.n)
        ]

        total = sum(pressure)
        now = time.monotonic()

        # STEP 3: 접촉하지 않은 상태
        if total < self.min_sum:
            if self.reference is not None:
                print("\n[CONTACT RELEASED]", flush=True)

            self.reference = None
            self.stable_cops.clear()
            self.moving_frames = 0
            self.alerted = False

            if now - self.last_print >= 1.0:
                print(f"[NO CONTACT] sum={total:.0f}",
                      flush=True)
                self.last_print = now
            return

        # STEP 4: 압력 중심 계산
        weighted_x = 0.0
        weighted_y = 0.0

        for row in range(self.rows):
            for col in range(self.cols):
                i = row * self.cols + col
                p = pressure[i]

                weighted_x += p * col
                weighted_y += p * row

        x = weighted_x / total
        y = weighted_y / total

        # STEP 5: 안정적인 최초 접촉 중심 저장
        if self.reference is None:
            self.stable_cops.append((x, y))

            if len(self.stable_cops) == 5:
                avg_x = sum(p[0] for p in self.stable_cops) / 5
                avg_y = sum(p[1] for p in self.stable_cops) / 5

                max_deviation = max(
                    math.hypot(px - avg_x, py - avg_y)
                    for px, py in self.stable_cops
                )

                if max_deviation <= 0.25:
                    self.reference = (avg_x, avg_y)

                    print(
                        f"\n[REFERENCE SET] "
                        f"CoP=({avg_x:.2f}, {avg_y:.2f})",
                        flush=True
                    )

            if now - self.last_print >= 1.0:
                print(
                    f"[STABILIZING] "
                    f"CoP=({x:.2f},{y:.2f}) "
                    f"sum={total:.0f}",
                    flush=True
                )
                self.last_print = now

            return

        # STEP 6: 최초 접촉 중심과 비교
        ref_x, ref_y = self.reference

        shift = math.hypot(
            x - ref_x,
            y - ref_y
        )

        if shift >= self.slip_threshold:
            self.moving_frames += 1
        else:
            self.moving_frames = 0

        # STEP 7: 3프레임 연속 초과 시 경고
        if (
            self.moving_frames >= self.confirm_frames
            and not self.alerted
        ):
            print("\n[CONTACT SHIFT DETECTED]", flush=True)
            print(
                f"Reference=({ref_x:.2f},{ref_y:.2f})",
                flush=True
            )
            print(
                f"Current=({x:.2f},{y:.2f})",
                flush=True
            )
            print(
                f"Shift={shift:.2f} cells",
                flush=True
            )
            print(
                "접촉 중심 이동 감지 - 실제 미끄러짐 확정 아님",
                flush=True
            )

            self.alerted = True

        # STEP 8: 약 1초마다 측정값 출력
        if now - self.last_print >= 1.0:
            print(
                f"CoP=({x:.2f},{y:.2f}) | "
                f"sum={total:.0f} | "
                f"shift={shift:.2f} | "
                f"frames={self.moving_frames}",
                flush=True
            )
            self.last_print = now


def main():
    rclpy.init()
    node = TactileCoPTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
