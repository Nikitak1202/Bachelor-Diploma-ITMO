# AGENTS Guide: Bachelor-Diploma-ITMO

## Purpose
This repository contains a ROS 2 simulation stack for an omnidirectional robot in an apartment world, including:
- environment orchestration,
- robot spawning and sensor/navigation bridges,
- dynamic target and obstacle simulation,
- Nav2 local control with MPPI.

This file is the top-level navigation map for AI agents and links to package-level guides with deeper implementation details.

## Repository Structure
- `src/apartment_sim`: Gazebo-side dynamic entities (wandering target and obstacles)
  and Gazebo ground-truth pose publisher used by offline analysis.  
  See: [`src/apartment_sim/AGENTS_apartment_sim.md`](src/apartment_sim/AGENTS_apartment_sim.md)
- `src/omni_robot`: Robot model, launch files, Nav2 integration, and target-tracking nodes.  
  See: [`src/omni_robot/AGENTS_omni_robot.md`](src/omni_robot/AGENTS_omni_robot.md)
- `src/nav2_mppi_controller`: C++ Nav2 MPPI controller plugin (local planner).  
  See: [`src/nav2_mppi_controller/AGENTS_nav2_mppi_controller.md`](src/nav2_mppi_controller/AGENTS_nav2_mppi_controller.md)
- `worlds`: Gazebo world files (`apartment.world`).
- `scripts`: Local orchestration (`run.sh`, `tmux-cfg.yml`) for Docker + tmux runtime,
  plus offline statistics (`statistics.py`, `bag_statistics_lib.py`, `statistics_all.py`,
  `requirements_stats.txt`).
  Root `Makefile`: `make run` → `scripts/run.sh`; `make delete-stats` → `scripts/delete_stats.sh`;
  `make plot` → `scripts/statistics.py`; `make plot-all` → `scripts/statistics_all.py`.
- `logs`: Runtime logs written by tmux panes and ROS/Gazebo processes.
  Sub-paths used by the recording/analysis pipeline:
  - `logs/rosbag/run_<ts>` — rosbag2 recordings produced by the `bag_record` tmux window
    (`<ts>` is in Europe/Moscow timezone)
    (each run must contain `metadata.yaml` after a clean stop; `scripts/statistics.py`
    skips incomplete folders and picks the latest valid bag);
  - `logs/stats/run_<ts>`  — PNG plots (`01`–`04`) from `scripts/statistics.py`
    (`/target_nav_mode`, `/target_visible`, `/scan`, `/cmd_vel` required).
  - `logs/stats/all_runs/` — aggregate plots (`all_01`–`all_04`, including safety event counts).
- `docs`: LaTeX report sources.
  - `docs/diploma_report`: final ВКР (bachelor thesis) sources, built with `pdflatex + bibtex` (`main.tex`, `include/`, `parts/`, `references.bib`, `gost71u.bst`). Compilation: `pdflatex main && bibtex main && pdflatex main && pdflatex main`. Images are referenced from `../images/` via `\graphicspath`. Sections currently marked as placeholders (experimental metrics, title-page metadata such as направление/программа) must be filled from ИСУ/ИТМО data before submission.
  - `docs/practice_reports`: earlier practice reports used as a source of theory and implementation descriptions for the diploma.
  - `docs/images`, `docs/literature`: shared figures and cited literature.

## Runtime Orchestration Flow
1. `scripts/run.sh` deletes **only loose files** directly under `logs/` (preserves `logs/rosbag/`,
   `logs/stats/`, etc.), rebuilds containers, and starts `docker-compose`.
2. `docker-compose.yml` mounts `src`, `worlds`, `scripts`, and `logs` into `/ros2_ws`.
3. `scripts/tmux-cfg.yml` launches:
   - Gazebo with apartment world,
   - isolated Gazebo and RViz visualization stacks (separate Xvfb/x11vnc/noVNC endpoints),
   - dynamic obstacles/target controller,
   - robot spawn,
   - Nav2 + MPPI stack,
   - target detector + navigation bridge,
   - Gazebo ground-truth pose publisher (`gazebo_truth_publisher`),
   - rosbag2 recorder for offline statistics (window `bag_record`,
     output to `logs/rosbag/run_<timestamp>`).
   - Browser access: Gazebo at `http://localhost:6080/vnc.html`, RViz at `http://localhost:8080/vnc.html`.
4. Navigation commands flow through topic/TF bridges in `omni_robot`.
5. MPPI controller from `nav2_mppi_controller` computes local trajectories in Nav2.
6. After the container exits, `python3 scripts/statistics.py` (host-side)
   reads the latest bag into `logs/stats/run_<timestamp>` (four figures per run).
   `python3 scripts/statistics_all.py` aggregates all valid bags into `logs/stats/all_runs/`
   (mode shares, visibility, safety event counts, control smoothness).

## Package Interaction Contracts
- `apartment_sim` provides moving entities in Gazebo using `/spawn_entity` and per-entity `/cmd_vel`.
- `omni_robot` bridges simulator-specific topics to Nav2-friendly interfaces (`/cmd_vel`, `/scan`, TF), detects the target, and sends `NavigateToPose` goals.
  Canonical target-tracking topics are `/target_visible` and `/target_pose`; RViz/debug outputs are `/target_marker` and `/omni_robot/camera/image_raw/target_status`.
- `nav2_mppi_controller` implements the `nav2_core::Controller` plugin used by `omni_robot/config/omni_nav2_params.yaml`.

## Working Rules for AI Agents
- Keep package boundaries strict:
  - world entity behavior in `apartment_sim`,
  - robot I/O bridges and pursuit logic in `omni_robot`,
  - local planner algorithm internals in `nav2_mppi_controller`.
- Preserve topic and frame consistency (`odom`, `base_link`, `/scan`, `/cmd_vel`, `/omni_robot/*`) unless the change explicitly includes full migration updates.
- For planning architecture, keep global planning map-centric (`map`) and local control odom-centric (`odom`) unless explicitly re-scoping the full stack.
- Prefer parameterized behavior changes over hardcoded constants.
- Validate changes from the package where they were made, then validate end-to-end launch composition.

## Quick Entry Points
- Full stack orchestration: `scripts/run.sh`
- Robot spawn launch: `src/omni_robot/launch/spawn_robot.launch.py`
- Nav2 + bridge launch: `src/omni_robot/launch/omni_nav2.launch.py`
- Dynamic world entities: `src/apartment_sim/apartment_sim/obstacles_controller.py`
- Gazebo ground-truth poses: `src/apartment_sim/apartment_sim/gazebo_truth_publisher.py`
- MPPI plugin entry: `src/nav2_mppi_controller/src/controller.cpp`
- Offline statistics (host): `scripts/statistics.py`, `scripts/statistics_all.py`
  (deps in `scripts/requirements_stats.txt`; bags without embedded message definitions use
  `Config.rosbag_default_typestore` as the rosbags fallback).
