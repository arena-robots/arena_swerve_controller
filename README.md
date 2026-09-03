# arena_swerve_controller

A `ros2_control` plugin for 4-wheel independent steering (swerve) chassis.

Consumes `geometry_msgs/Twist` (or `TwistStamped`) on the controller's
reference topic, splits the body twist into a per-wheel velocity vector at
each hub position, and commands each wheel pair as
`(traction velocity, steering angle)`. Inverse kinematics from the measured
state populates `nav_msgs/Odometry`.

## Plugin

```
arena_swerve_controller/SwerveController
```

inherits `controller_interface::ControllerInterface`.

## Interfaces

| | wheel joint | steering joint |
|---|---|---|
| command | `velocity` | `position` |
| state   | `velocity` | `position` |

Four of each, in the order declared by `wheel_joint_names` /
`steering_joint_names`. Joint indices must align with `wheel_positions_x` /
`wheel_positions_y`.

## Parameters

See [`src/arena_swerve_controller_parameters.yaml`](src/arena_swerve_controller_parameters.yaml).
Minimum required: `wheel_joint_names`, `steering_joint_names`, `wheel_radius`,
`wheel_positions_x`, `wheel_positions_y`.

## Topics

| topic | direction | type | when |
|---|---|---|---|
| `~/cmd_vel`   | sub | `geometry_msgs/Twist`        | `use_stamped_vel: false` |
| `~/cmd_vel`   | sub | `geometry_msgs/TwistStamped` | `use_stamped_vel: true`  |
| `~/odometry`  | pub | `nav_msgs/Odometry`          | `publish_odom: true`     |
| `/tf`         | pub | `tf2_msgs/TFMessage`         | `publish_tf: true`       |

## Steering behaviour

Each wheel may drive reversed with its heading shifted by pi when that is the shorter
steering move and the shifted heading stays within `max_steering_angle`. The drive sense is
sticky: a wheel keeps it until the other sense saves more than `flip_hysteresis` (default
0.35 rad) of travel, which stops a command near the pi/2 boundary from flipping every tick.
Traction is scaled by the cosine of the remaining steering error and cut entirely beyond
`traction_gate_angle` (default 60 degrees), so wheels are not driven against each other while
they are still turning toward a new heading.

## Example: rbvogui control.yaml

```yaml
/**:
  controller_manager:
    ros__parameters:
      use_sim_time: true
      update_rate: 50

      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

      robotnik_base_controller:
        type: arena_swerve_controller/SwerveController

  robotnik_base_controller:
    ros__parameters:
      use_sim_time: true

      # Order: front_right, front_left, back_right, back_left
      wheel_joint_names:
        - robot_front_right_wheel_joint
        - robot_front_left_wheel_joint
        - robot_back_right_wheel_joint
        - robot_back_left_wheel_joint
      steering_joint_names:
        - robot_front_right_steering_joint
        - robot_front_left_steering_joint
        - robot_back_right_steering_joint
        - robot_back_left_steering_joint

      wheel_radius: 0.1125
      # wheel_base 0.76 m, track_width 0.4745 m, base_link at geometric center.
      wheel_positions_x: [ 0.38,  0.38, -0.38, -0.38]
      wheel_positions_y: [-0.23725, 0.23725, -0.23725, 0.23725]

      max_steering_angle: 2.8
      allow_reverse_drive: true
      cmd_vel_timeout: 0.5
      use_stamped_vel: true

      publish_odom: true
      publish_tf: false
      odom_frame_id: odom
      base_frame_id: robot_base_footprint
```
