# AGENTS: omni_robot

## Role

Robot description (URDF/Xacro), Gazebo spawn, Nav2 + SLAM bringup, topic and TF bridges between Gazebo namespaces and Nav2 defaults, and the target-chase loop (vision + Nav2 goals + spin when lost).

## Main files

- `urdf/omni_robot.urdf.xacro` — chassis, lidar, camera plugins for Gazebo.
- `launch/spawn_robot.launch.py` — writes URDF to `/tmp`, spawns `omni_robot` in Gazebo.
- `launch/omni_nav2.launch.py` — includes `nav2_bringup`, starts bridges and target nodes, sets `FollowPath` to the MPPI plugin from `nav2_mppi_controller`.
- `config/omni_nav2_params.yaml` — Nav2, controller, SLAM-related parameters; MPPI critic weights live here.
- `config/slam.yaml` — slam_toolbox online mapping.
- `omni_robot/scan_bridge.py` — `/omni_robot/scan` to `/scan`.
- `omni_robot/cmd_vel_bridge.py` — `/cmd_vel` to `/omni_robot/cmd_vel`.
- `omni_robot/odom_tf_bridge.py` — publishes TF `odom` → `base_link` from Gazebo odom.
- `omni_robot/target_detector.py` — blue blob in camera + lidar hints; publishes `/target_visible`, `/target_pose`, markers.
- `omni_robot/target_nav_bridge.py` — `NavigateToPose` updates while visible; spin / last-pose behavior when not.

## Topic contract

Bridges must keep `/scan` and `/cmd_vel` what Nav2 and slam_toolbox expect. Target stack publishes `/target_visible` (bool), `/target_pose`, `/target_nav_mode` (string). Changing frame IDs requires coordinated edits in bridges, YAML, and SLAM.

## Rules for changes

Prefer launch arguments and YAML parameters over literals in Python. If you change odometry or base link naming, update bridges, `omni_nav2_params.yaml`, and SLAM frames in one change set. Target loss is normal; keep timeouts and spin behavior coherent with Nav2 BT if you touch the bridge.

## Validation

`ros2 launch omni_robot spawn_robot.launch.py` and `ros2 launch omni_robot omni_nav2.launch.py` separately, then full `make run`. Check TF tree and that goals flow when the target is visible.

Back: `../../AGENTS.md`
