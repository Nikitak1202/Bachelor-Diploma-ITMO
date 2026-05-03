# AGENTS: nav2_mppi_controller

## Role

C++ package implementing the Nav2 local controller plugin `nav2_mppi_controller::MPPIController`: trajectory sampling, critic scoring, command output. No Gazebo or camera code belongs here.

## Layout

- `src/controller.cpp` — Nav2 controller plugin entry.
- `src/optimizer.cpp`, `include/.../optimizer.hpp` — MPPI rollouts and update step.
- `src/critic_manager.cpp` — loads and runs critics.
- `src/critics/*.cpp` — individual cost terms (path, obstacles, goal, custom critics such as target-bearing helpers).
- `mppic.xml`, `critics.xml` — pluginlib registration; names here must match YAML `plugin:` entries.

## Configuration

Tuning for this repo is primarily in `omni_robot/config/omni_nav2_params.yaml` under `controller_server.ros__parameters.FollowPath` (plugin type, critic lists, numeric weights). Change YAML first; modify C++ only when behavior cannot be achieved by parameters.

## Rules for changes

Preserve exported plugin class names and XML unless you intentionally break ABI and update all YAML references. Keep this package free of simulator-specific topic names; bridging stays in `omni_robot`. After critic or sampling changes, run `colcon test` for this package and a short full-stack sim for regressions.

## Validation

`colcon build --packages-select nav2_mppi_controller` and `colcon test --packages-select nav2_mppi_controller`, then exercise `make run` and watch local plan stability near obstacles and the target.

Back: `../../AGENTS.md`
