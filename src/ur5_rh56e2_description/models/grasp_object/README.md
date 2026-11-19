# Supported-object grasp trial

Use two separate Gazebo models: a static 9 cm high support and a dynamic
50 g cylinder (3 cm diameter, 24 cm long). Neither a fixed joint nor a base
connects the cylinder to the support. Deleting only the support tests whether
the hand can hold the cylinder against gravity.

Remove old grasp pillars in Gazebo before this trial. Open the hand first:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rh56e2_control hand_command_node --ros-args -p mode:=open
```

Stop the command node with Ctrl+C when open. Create the support first, then
the object. Keep x/y identical; tune both together if the initial placement
misses the space between thumb and fingers.

```bash
ros2 run ros_gz_sim create --world default \
  --file "$HOME/ros2_ws/src/ur5_rh56e2_description/models/grasp_support/model.sdf" \
  --name grasp_support_1 -x 0.91 -y 0.23 -z 0

ros2 run ros_gz_sim create --world default \
  --file "$HOME/ros2_ws/src/ur5_rh56e2_description/models/grasp_object/model.sdf" \
  --name grasp_object_1 -x 0.91 -y 0.23 -z 0.211
```

The object's bottom starts 1 mm above the support and settles onto it.
Close the open hand with a smooth 4 second finger target ramp. The two thumb
joints start 2 seconds after the other fingers and close over 4 seconds:

```bash
ros2 run rh56e2_control hand_command_node --ros-args \
  -p mode:=close -p close_duration:=4.0 \
  -p thumb_delay:=2.0 -p thumb_close_duration:=4.0 \
  -p use_sim_time:=true
```

The controller must already be active and in MOVEJ. Duration describes the
target ramp in ROS time, not guaranteed physical joint motion. With
`use_sim_time:=true`, the ramp follows Gazebo's clock and pauses with simulation.
Without that parameter it follows wall time. The delay reduces early thumb
contact but does not detect contact or guarantee a stable grasp; if fingers
push the object first, placement and timing still need adjustment. Set
`thumb_delay:=0.0` and equal durations to close all fingers together. Leave the
hand closed and remove only the support once the object has settled:

```bash
gz service -s /world/default/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 5000 --req 'name: "grasp_support_1", type: MODEL'
```

Expect `data: true`. This deletes the support, including its collision shape.
Alternatively delete the support model in Gazebo's entity tree.
Observe for several seconds of simulation time: remaining suspended is
evidence of a hold; falling or sliding means the grasp needs adjustment.
For a stronger check, open the hand afterwards and confirm the cylinder falls.
If it stays suspended even with the hand open, inspect other contacts or a
paused simulation. Do not infer a successful grasp just from curled fingers.
