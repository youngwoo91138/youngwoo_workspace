# RH56E2 촉각 미끄러짐 테스트

Inspire RH56E2-2R-T1의 촉각 패드로 파지 중 미끄러짐을 감지합니다. 파지와 힘 영점 보정에는 다섯 손가락을 사용하고, 미끄러짐은 검지·중지·약지의 압력 중심(CoP)과 압력 합으로 판단합니다.

## 설치 및 빌드

Ubuntu 24.04와 ROS 2 Jazzy가 설치된 환경에서 실행합니다. 처음 설정하는 환경이라면 `sudo rosdep init`으로 rosdep을 초기화합니다.

```bash
sudo apt install git python3-colcon-common-extensions python3-rosdep
git clone https://github.com/leeheejoon/rh56e2_gazebo_project.git
cd rh56e2_gazebo_project
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths $(colcon list --packages-up-to rh56e2_control --paths-only) \
  --ignore-src --rosdistro jazzy --skip-keys lina_planning \
  -t build -t buildtool -t build_export -t buildtool_export -t exec -y
colcon build --symlink-install --packages-up-to rh56e2_control --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`lina_planning`은 이 테스트에서 사용하지 않는 선택적 궤적 계획 의존성입니다.

## 실물 테스트

1. 촉각 센서가 있는 RH56E2 T1과 USB–RS485 어댑터를 연결합니다. 아래 명령의 `serial_port`를 실제 장치 경로로 바꿉니다. `ls /dev/serial/by-id/`로 고정 경로를 확인할 수 있습니다. 포트 접근 권한이 필요하면 `sudo usermod -aG dialout "$USER"` 실행 후 다시 로그인합니다.
2. 저장소에서 첫 번째 터미널을 열어 하드웨어를 시작합니다. 기본값은 오른손(ID 1), 115200 baud입니다. 왼손(ID 2)은 `direction:=1`을 추가합니다.

   ```bash
   source install/setup.bash
   ros2 launch rh56e2_control real.launch.py serial_port:=/dev/ttyUSB0
   ```

3. 첫 번째 터미널에서 `FSM initialization complete.`를 확인하고, 저장소에서 두 번째 터미널을 열어 테스트를 시작합니다. 아래 명령은 초기 파지를 수행하며 미끄러짐이 감지되면 경고만 출력합니다.

   ```bash
   source install/setup.bash
   ros2 run rh56e2_control hand_command_node --ros-args \
     -p object_name:=sample -p enable_auto_regrasp:=false
   ```

4. 손을 비운 채 힘 센서 영점 보정을 기다립니다. `[FORCE ZERO DONE]` 이후 손가락 경로를 비운 상태에서 테스트 물체를 지그에 고정해 파지합니다. `ALL 5 FINGERS LOCKED`와 `CoP baseline ready`가 표시되면 물체에 작은 하중을 가해 미끄러짐을 유도합니다.
5. `[SLIP SUSPECTED]` 로그에서 감지된 손가락과 원인을 확인합니다. 기본값은 CoP가 0.2셀 이상 이동하거나 압력 합이 기준보다 20% 이상 감소한 상태가 3프레임 연속 관측될 때 경고를 출력합니다. 자동 재파지까지 시험하려면 두 번째 명령에서 `enable_auto_regrasp:=true`를 지정합니다.

촉각 데이터 수신을 확인하려면 `ros2 topic echo /hand/tactile/index_pad --once --qos-reliability best_effort`를 실행합니다.

## Gazebo

Gazebo에서는 UR5와 RH56E2 모델 및 관절 컨트롤러를 확인할 수 있습니다. 위 미끄러짐 테스트에는 실물 T1의 촉각 토픽이 필요합니다.

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ur-description \
  ros-jazzy-position-controllers
colcon build --symlink-install --packages-up-to rh56e2_control ur5_rh56e2_description \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
ros2 launch rh56e2_control sim.launch.py
```
