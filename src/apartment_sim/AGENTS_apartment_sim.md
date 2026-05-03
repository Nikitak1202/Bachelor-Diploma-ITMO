# AGENTS: apartment_sim

## Role

ROS 2 package for Gazebo-only dynamics: spawn a blue cylindrical target and red obstacle cylinders, publish `cmd_vel` to each model. Does not implement robot navigation or the MPPI controller.

## Main code

- `apartment_sim/obstacles_controller.py` — calls `gazebo_msgs/srv/SpawnEntity`, builds simple SDF strings, sends random planar velocities on `/<model_name>/cmd_vel`.
- `launch/launch.py` — optional standalone launch for the controller with parameters.

## External dependencies

- Gazebo must provide `/spawn_entity`.
- Model names must stay consistent with anything that parses Gazebo state: target is `target`, obstacles `obstacle_0` … `obstacle_N`, robot `omni_robot` (used by other packages when matching `gazebo/model_states` names).

## Rules for changes

Keep spawn layout and speeds in ROS parameters (`declare_parameter`) so scenarios stay tunable without code edits. If you rename models, update every consumer (camera-based target is still `target` in the world).

## Validation

Run the full stack from the repo root (`make run`) and confirm entities appear and move. If spawn fails, check Gazebo is up before the controller’s delayed start (orchestrated in `scripts/tmux-cfg.yml`).

Back: `../../AGENTS.md`
