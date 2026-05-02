# AGENTS Guide: apartment_sim

## Scope
`apartment_sim` is the world-entity package. Its purpose is to create and animate non-robot actors in Gazebo:
- one blue moving target (`target`),
- multiple red moving obstacles (`obstacle_*`).

This package does not own robot navigation or local planner internals.

Back to root guide: [`../../AGENTS.md`](../../AGENTS.md)

## Main Components
- `apartment_sim/obstacles_controller.py`
  - ROS 2 node that calls `/spawn_entity`,
  - generates SDF cylinders for target and obstacles,
  - publishes random planar velocities to each entity namespace.
- `apartment_sim/gazebo_truth_publisher.py`
  - ROS 2 node that subscribes to `/gazebo/model_states`,
  - re-publishes target and robot poses in the SLAM `map` frame
    (`/ground_truth/target_pose`, `/ground_truth/robot_pose`),
  - applies a static `world -> map` offset matching SLAM init at the
    robot spawn pose (parameters `robot_spawn_x/y/yaw`),
  - matches entities by the **last** `::` segment of `model_states` names so Gazebo-scoped
    names still resolve to `target` / `omni_robot`.
- `launch/launch.py`
  - launches `obstacles_controller` with deterministic spawn/behavior parameters.
- `setup.py` / `package.xml`
  - package metadata and console script registration.

## Interfaces Owned by This Package
- **Service dependency**
  - `/spawn_entity` (`gazebo_msgs/srv/SpawnEntity`) provided by Gazebo.
- **Subscribed topics**
  - `/gazebo/model_states` (`gazebo_msgs/msg/ModelStates`) consumed by
    `gazebo_truth_publisher`.
- **Published topics**
  - `/<entity_name>/cmd_vel` (`geometry_msgs/msg/Twist`) for each spawned model.
  - `/ground_truth/target_pose`, `/ground_truth/robot_pose`
    (`geometry_msgs/msg/PoseStamped`) by `gazebo_truth_publisher`,
    used by offline statistics in `scripts/statistics.py`.
- **Entity conventions**
  - target model name: `target` (blue),
  - obstacle model names: `obstacle_0..N` (red),
  - robot model name: `omni_robot` (matched by ground-truth publisher).

## Interaction With Other Packages
- `omni_robot/target_detector` relies on visual detectability of the blue `target` in camera frames.
- Nav2 costmaps consume LiDAR observations from spawned moving obstacles indirectly via robot sensors.
- Runtime launch order is orchestrated by `scripts/tmux-cfg.yml` and not by this package itself.

## Change Guidelines
- Keep spawn behavior parameter-driven (`declare_parameter`) to avoid hardcoded scenario lock-in.
- Preserve deterministic spawn semantics (`target_spawn_*`, `obstacle_spawn_positions`) unless the scenario design is intentionally changed.
- If entity naming changes, update all downstream consumers expecting `target` and `obstacle_*`.
- Keep Gazebo plugin settings (`planar_move`) aligned with expected `cmd_vel` and odometry topic names.

## Validation Focus
- Package-level:
  - `ros2 launch apartment_sim launch.py`
- Integration-level:
  - verify entities spawn and move in Gazebo,
  - verify target remains detectable by `omni_robot/target_detector`.
