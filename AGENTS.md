# AGENTS Guide: Bachelor-Diploma-ITMO

## Purpose
This repository contains a ROS 2 simulation stack for an omnidirectional robot in an apartment world, including:
- environment orchestration,
- robot spawning and sensor/navigation bridges,
- dynamic target and obstacle simulation,
- Nav2 local control with MPPI.

This file is the top-level navigation map for AI agents and links to package-level guides with deeper implementation details.

## Repository Structure
- `src/apartment_sim`: Gazebo-side dynamic entities (wandering target and obstacles).  
  See: [`src/apartment_sim/AGENTS_apartment_sim.md`](src/apartment_sim/AGENTS_apartment_sim.md)
- `src/omni_robot`: Robot model, launch files, Nav2 integration, and target-tracking nodes.  
  See: [`src/omni_robot/AGENTS_omni_robot.md`](src/omni_robot/AGENTS_omni_robot.md)
- `src/nav2_mppi_controller`: C++ Nav2 MPPI controller plugin (local planner).  
  See: [`src/nav2_mppi_controller/AGENTS_nav2_mppi_controller.md`](src/nav2_mppi_controller/AGENTS_nav2_mppi_controller.md)
- `worlds`: Gazebo world files (`apartment.world`).
- `scripts`: Local orchestration (`run.sh`, `tmux-cfg.yml`) for Docker + tmux runtime.
- `logs`: Runtime logs written by tmux panes and ROS/Gazebo processes.
- `docs`: LaTeX report sources (project documentation).

## Runtime Orchestration Flow
1. `scripts/run.sh` cleans logs, rebuilds containers, and starts `docker-compose`.
2. `docker-compose.yml` mounts `src`, `worlds`, `scripts`, and `logs` into `/ros2_ws`.
3. `scripts/tmux-cfg.yml` launches:
   - Gazebo with apartment world,
   - isolated Gazebo and RViz visualization stacks (separate Xvfb/x11vnc/noVNC endpoints),
   - dynamic obstacles/target controller,
   - robot spawn,
   - Nav2 + MPPI stack,
   - target detector + navigation bridge.
   - Browser access: Gazebo at `http://localhost:6080/vnc.html`, RViz at `http://localhost:8080/vnc.html`.
4. Navigation commands flow through topic/TF bridges in `omni_robot`.
5. MPPI controller from `nav2_mppi_controller` computes local trajectories in Nav2.

## Package Interaction Contracts
- `apartment_sim` provides moving entities in Gazebo using `/spawn_entity` and per-entity `/cmd_vel`.
- `omni_robot` bridges simulator-specific topics to Nav2-friendly interfaces (`/cmd_vel`, `/scan`, TF), detects the target, and sends `NavigateToPose` goals.
- `nav2_mppi_controller` implements the `nav2_core::Controller` plugin used by `omni_robot/config/omni_nav2_params.yaml`.

## Working Rules for AI Agents
- Keep package boundaries strict:
  - world entity behavior in `apartment_sim`,
  - robot I/O bridges and pursuit logic in `omni_robot`,
  - local planner algorithm internals in `nav2_mppi_controller`.
- Preserve topic and frame consistency (`odom`, `base_link`, `/scan`, `/cmd_vel`, `/omni_robot/*`) unless the change explicitly includes full migration updates.
- Prefer parameterized behavior changes over hardcoded constants.
- Validate changes from the package where they were made, then validate end-to-end launch composition.

## Quick Entry Points
- Full stack orchestration: `scripts/run.sh`
- Robot spawn launch: `src/omni_robot/launch/spawn_robot.launch.py`
- Nav2 + bridge launch: `src/omni_robot/launch/omni_nav2.launch.py`
- Dynamic world entities: `src/apartment_sim/apartment_sim/obstacles_controller.py`
- MPPI plugin entry: `src/nav2_mppi_controller/src/controller.cpp`
