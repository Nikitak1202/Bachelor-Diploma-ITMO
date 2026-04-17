# AGENTS Guide: nav2_mppi_controller

## Scope
`nav2_mppi_controller` is the C++ Nav2 local planner plugin package implementing MPPI (Model Predictive Path Integral control).

It owns trajectory sampling, scoring (critics), and command selection logic for `nav2_core::Controller` integration.

Back to root guide: [`../../AGENTS.md`](../../AGENTS.md)

## Main Components
- `src/controller.cpp`
  - plugin entrypoint implementing `nav2_core::Controller` behavior.
- `src/optimizer.cpp` + `include/nav2_mppi_controller/optimizer.hpp`
  - MPPI optimization loop over sampled trajectories.
- `src/critic_manager.cpp`
  - critic lifecycle and scoring orchestration.
- `src/critics/*.cpp` + `include/nav2_mppi_controller/critics/*.hpp`
  - pluggable cost terms (goal, path, obstacles, constraints, etc.).
- `src/path_handler.cpp`, `src/trajectory_visualizer.cpp`, `src/noise_generator.cpp`
  - support modules for path preparation, debug visualization, and sampling noise.
- `mppic.xml` and `critics.xml`
  - pluginlib exports for controller and critic plugins.

## Integration Contract
- Exported plugin used by Nav2 as:
  - `plugin: "nav2_mppi_controller::MPPIController"`
- Configured in this repository via:
  - `src/omni_robot/config/omni_nav2_params.yaml` under `controller_server.FollowPath`.

## Tests and Benchmarks
- Unit/integration tests in `test/*.cpp`.
- Optional performance benchmarks in `benchmark/*.cpp`.
- Build and test are CMake-based (`ament_cmake`), unlike Python packages in this repo.

## Change Guidelines
- Treat this package as algorithm/core-planner layer; keep simulator-specific topic bridging out of this package.
- Preserve plugin ABI and exported names (`mppic.xml`, `critics.xml`) unless intentionally performing a breaking change.
- When tuning default behavior, prefer parameter changes in `omni_nav2_params.yaml` first; modify controller internals only when tuning is insufficient.
- For critic changes, validate interactions with costmap inflation and path-tracking critics as they are tightly coupled.

## Validation Focus
- Package-level:
  - `colcon build --packages-select nav2_mppi_controller`
  - `colcon test --packages-select nav2_mppi_controller`
- Integration-level:
  - launch `omni_robot` Nav2 stack and verify controller stability/trajectory quality in the apartment world.
