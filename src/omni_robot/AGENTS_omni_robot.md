# AGENTS Guide: omni_robot

## Scope
`omni_robot` owns the simulated robot package:
- robot model (URDF/Xacro),
- launch composition for spawn and Nav2 bringup,
- topic/TF bridge nodes between Gazebo and Nav2 conventions,
- target pursuit pipeline (detect target -> publish pose -> send Nav2 goals).

Back to root guide: [`../../AGENTS.md`](../../AGENTS.md)

## Main Components
- `urdf/omni_robot.urdf.xacro`
  - robot geometry and sensors for Gazebo.
- `launch/spawn_robot.launch.py`
  - generates URDF and spawns robot as `omni_robot` entity.
- `launch/omni_nav2.launch.py`
  - includes `nav2_bringup`,
  - starts bridges: `scan_bridge`, `odom_tf_bridge`, `cmd_vel_bridge`,
  - publishes static alias `base_link -> base_footprint`.
- `config/omni_nav2_params.yaml`
  - Nav2/SLAM/server parameters,
  - configures `FollowPath` plugin as `nav2_mppi_controller::MPPIController`.
- `omni_robot/*.py` runtime nodes:
  - `scan_bridge.py`: `/omni_robot/scan -> /scan`
  - `cmd_vel_bridge.py`: `/cmd_vel -> /omni_robot/cmd_vel`
  - `odom_tf_bridge.py`: TF `odom -> base_link` from `/omni_robot/odom`
  - `target_detector.py`: camera-based blue-target detection and pose estimation
  - `target_nav_bridge.py`: sends/cancels `NavigateToPose` goals based on visibility

## Key Interfaces and Dataflow
- **Sensor/nav adaptation**
  - Gazebo topics are adapted to Nav2-default topics and TF tree.
- **Target tracking**
  - `target_detector` publishes:
    - `/target_tracker/target_visible`,
    - `/target_tracker/target_pose`,
    - `/target_tracker/range_m`,
    - `/target_tracker/horizontal_error`.
  - `target_nav_bridge` consumes visibility + pose and drives `/navigate_to_pose` action goals.

## Dependencies on Other Packages
- Consumes moving entities produced by `apartment_sim`.
- Depends on MPPI plugin exported by `nav2_mppi_controller`.
- Uses `nav2_bringup`, `slam_toolbox`, Gazebo ROS, and TF2 runtime packages.

## Change Guidelines
- Keep topic/frame contracts stable unless all related launch/config/node files are updated together.
- Prefer launch arguments and ROS parameters over hardcoded runtime constants.
- For target-tracking logic updates, maintain robustness to temporary target loss (`target_nav_bridge` cancellation behavior).
- If changing robot frames or odometry topics, update:
  - bridges,
  - Nav2 parameter file,
  - SLAM/BT frames in the same change.

## Validation Focus
- Package-level:
  - `ros2 launch omni_robot spawn_robot.launch.py`
  - `ros2 launch omni_robot omni_nav2.launch.py use_composition:=False`
- Integration-level:
  - verify TF tree includes `odom -> base_link` and `base_link -> base_footprint`,
  - verify `/scan` and `/cmd_vel` relays work with Nav2,
  - verify target detection updates Nav2 goals while visible.
