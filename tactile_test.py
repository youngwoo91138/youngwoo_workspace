
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray


class TactileTest(Node):
    def __init__(self):
        super().__init__('tactile_test')

        self.baseline_samples = []
        self.baseline = None
        self.previous = None
        self.count = 0

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/hand/tactile/index_pad',
            self.callback,
            qos_profile_sensor_data
        )

        print("촉각 데이터 수신 대기...")
        print("처음 40개 샘플 동안 센서를 누르지 마세요.")

    def callback(self, msg):
        data = list(msg.data)

        if len(data) != 80:
            print(f"데이터 개수 오류: {len(data)}")
            return

        # STEP 1: 초기 기준값 측정
        if self.baseline is None:
            self.baseline_samples.append(data)

            if len(self.baseline_samples) == 40:
                self.baseline = [
                    sum(sample[i] for sample in self.baseline_samples) / 40
                    for i in range(80)
                ]
                print("\n초기 보정 완료!")
                print("이제 검지 촉각 패드를 가볍게 눌러보세요.")

            return

        # STEP 2: 초기 기준값과 비교
        delta = [
            data[i] - self.baseline[i]
            for i in range(80)
        ]

        baseline_change = sum(abs(v) for v in delta)

        # STEP 3: 직전 측정값과 비교
        frame_change = 0.0

        if self.previous is not None:
            frame_change = sum(
                abs(data[i] - self.previous[i])
                for i in range(80)
            )

        self.previous = data.copy()
        self.count += 1

        # 약 1초마다 출력 (8Hz 가정)
        if self.count % 8 == 0:
            top = sorted(
                range(80),
                key=lambda i: abs(delta[i]),
                reverse=True
            )[:3]

            print("\n------ 촉각 측정 결과 ------")
            print(f"기준값 대비 총 변화: {baseline_change:.1f}")
            print(f"직전 프레임 대비 변화: {frame_change:.1f}")

            for i in top:
                row = i // 8
                col = i % 8
                print(
                    f"행 {row}, 열 {col}: "
                    f"원시값={data[i]:.1f}, "
                    f"보정 변화={delta[i]:+.1f}"
                )


def main():
    rclpy.init()
    node = TactileTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
