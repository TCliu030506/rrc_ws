# ur5_sim_servo_control

This package provides a small UR5 simulation bridge that converts a target pose into Servo twist commands.

The actual low-level motion control is handled by `moveit_servo`, which is the mature and stable open-source component used here.

## What it does

- Subscribes to a target pose topic, defaulting to `/admittance/cmd_pose`.
- Reads the current end-effector pose from TF, defaulting to `world -> tool0`.
- Publishes a bounded `TwistStamped` command to the Servo input topic, defaulting to `/servo_node/delta_twist_cmds`.

## Expected Servo side

Use a `moveit_servo` configuration that:

- accepts Cartesian twist commands on `~/delta_twist_cmds`
- publishes joint trajectories to the UR5 `joint_trajectory_controller`
- enables collision checking

The important parameters are usually:

- `command_in_type: speed_units`
- `cartesian_command_in_topic: ~/delta_twist_cmds`
- `command_out_type: trajectory_msgs/JointTrajectory`
- `command_out_topic: /joint_trajectory_controller/joint_trajectory`
- `move_group_name: ur_manipulator`
- `planning_frame: world`
- `ee_frame_name: tool0`

An UR5-oriented recommended config is shipped at:

- [config/ur5_moveit_servo_recommended.yaml](config/ur5_moveit_servo_recommended.yaml)

## Bridge launch

The bridge can be started standalone:

```bash
ros2 launch ur5_sim_servo_control ur5_pose_target_servo_bridge.launch.py
```

If you want the existing `ultra_scanning_sim` launch to use Servo instead of the IK+trajectory path, set `use_servo_bridge:=true` there.

## Build

```bash
colcon build --packages-select ur5_sim_servo_control
source install/setup.bash
```

## Run

```bash
ros2 launch ur5_sim_servo_control ur5_sim_servo_control.launch.py
```

## Notes

- This package does not replace `moveit_servo`; it only bridges your pose target to Servo.
- If your Servo node uses a different namespace, change `servo_twist_topic` accordingly.
- If your incoming target pose is not in `base_link`, update the publisher upstream or extend this bridge to subscribe to `PoseStamped`.
