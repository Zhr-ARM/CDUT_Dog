# sim_motor_controller

This package mirrors the real DM-J4340 controller ROS interface for simulation.

Motor mapping:

- Motor 1: `base_yaw_joint`
- Motor 2: `shoulder_pitch_joint`
- Motor 3: `elbow_pitch_joint`
- Motor 4: `wrist_pitch_joint`

Control topic:

```text
/dm_j4340/mit_commands
```

Message type:

```text
std_msgs/msg/Float32MultiArray
```

The command format is identical to the real controller:

```text
[p, v, kp, kd, t_ff] * motor_count
```

State and services are also mirrored:

```text
/dm_j4340/joint_states
/dm_j4340/status
/dm_j4340/enable_all
/dm_j4340/disable_all
/dm_j4340/set_zero_all
/dm_j4340/clear_faults_all
```

Run:

```bash
ros2 launch sim_motor_controller sim_motor_controller.launch.py
```
