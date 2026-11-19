# RH56E2 pillar grasp trial

The dynamic model has a 4 cm diameter, 40 cm long stem on a 14 cm diameter,
2 cm thick weighted base. The model origin is the bottom of the base.
Spawn at z=0.001, not z=0.21: the stem spans z=0.021 to z=0.421.
The base supports the object before the hand closes; gravity stays enabled.

With the initial UR5 pose in `ur5_rh56e2.urdf.xacro`, start with x=0.93,
y=0.23. This is a placement estimate, not a verified grasp pose.

After building, source `install/setup.bash` in both terminals and restart
`ros2 launch rh56e2_control sim.launch.py`. Wait for `FSM -> MOVEJ`.

Open the hand first (stop each command node with Ctrl+C before starting another):

```bash
ros2 run rh56e2_control hand_command_node --ros-args -p mode:=open
```

Spawn one pillar; use a new name for another trial if the old model still exists:

```bash
ros2 run ros_gz_sim create \
  --world default \
  --file "$HOME/ros2_ws/src/ur5_rh56e2_description/models/grasp_pillar/model.sdf" \
  --name grasp_pillar_1 \
  -x 0.93 -y 0.23 -z 0.001
```

Close gradually (default: open for 2 seconds, then close over 4 seconds):

```bash
ros2 run rh56e2_control hand_command_node
```

Adjust the target angles in radians, in this order: thumb_joint1, thumb_joint2,
index_joint, middle_joint, ring_joint, pinky_joint.

```bash
ros2 run rh56e2_control hand_command_node --ros-args \
  -p close_duration:=6.0 \
  -p 'target_positions:=[0.9, 0.45, 1.0, 1.0, 1.0, 1.0]'
```

Inspect the gap with the hand open before closing. If the fingers miss the
stem, adjust x/y by 0.005–0.01 m in the next trial. If the object tips or is
pushed away, reduce the closing angles and check initial overlap.
The node sends position targets; its completion log does not measure contact
or prove that the object is held. Confirm contact in Gazebo before attempting
lifting. This trial does not move the arm or implement force control.
