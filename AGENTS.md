# AGENTS: Bachelor-Diploma-ITMO

ROS 2 Humble: Gazebo apartment world, omnidirectional robot, Nav2 with custom MPPI plugin, moving target and obstacles, host-side rosbag plotting.

## Repo map

- `src/apartment_sim` — spawn and move non-robot models in Gazebo. See `src/apartment_sim/AGENTS_apartment_sim.md`.
- `src/omni_robot` — URDF, spawn + Nav2 launch, Gazebo-to-Nav2 bridges, target detection and goal bridge. See `src/omni_robot/AGENTS_omni_robot.md`.
- `src/nav2_mppi_controller` — C++ `nav2_core::Controller` MPPI plugin and critics. See `src/nav2_mppi_controller/AGENTS_nav2_mppi_controller.md`.
- `src/omni_behavior` — optional Nav2 BT condition plugin (unused in default chase path). See `src/omni_behavior/AGENTS_omni_behavior.md`.
- `worlds/` — Gazebo world file(s).
- `scripts/` — Docker/tmux (`run.sh`, `tmux-cfg.yml`, `tmux-cfg-headless.yml`, `container_main.sh`, `collect_stats.sh`), stats (`statistics.py`, `statistics_all.py`, `bag_statistics_lib.py`).
- `docs/diploma_report/` — LaTeX thesis (`pdflatex`, `bibtex`).

## Make targets

- `make run` — `scripts/run.sh`: `-it`, `OMNI_TMUX_ATTACH=1`; after **Ctrl+b d** (detach), `container_main.sh` sends **C-c** to the bag window, waits `OMNI_BAG_FINALIZE_SEC` (default 10, set in `run.sh`), **kill-server**, then **exit 0** so `docker-compose run --rm` removes the container; then `docker-compose down`. Use **`make plot`** on the bag under `logs/rosbag/`.
- `make clean` — `scripts/delete_stats.sh`: deletes every subdirectory of `logs/` (rosbag, stats, etc.); top-level files under `logs/` stay.
- `make stats` — runs `collect_stats.sh`: calls `make clean`, builds image once, headless batches with env `SIMS`, `PARALLEL`, `DURATION_SEC`, `BAG_FINALIZE_SEC`. Each simulation writes `logs/rosbag/runN` (`OMNI_BAG_RUN_NAME`). Ends with `plot-all` then `plot` (latest bag by mtime).
- `make plot` — latest **complete** bag under `logs/rosbag/` (max mtime among dirs with `metadata.yaml`): either `run_…` from `make run` or `runN` from `make stats`.
- `make plot RUN=N` — `logs/rosbag/runN`. Plain `make plot 1` is invalid; use `RUN=1`.
- `make plot-all` — aggregate figures into `logs/stats/all_runs/`; run order on axes: `run1`, `run2`, … then `run_…` and other names.

## Package boundaries

World-side motion and spawn only in `apartment_sim`. Robot IO, TF, target pipeline, Nav2 launch files in `omni_robot`. MPPI math and critics in `nav2_mppi_controller`. Do not move simulator-only hacks into the planner package or vice versa.

## Topics and frames (do not break casually)

Nav2 and bridges expect `/scan`, `/cmd_vel`, `odom`, `map`, `base_link`, `/target_visible`, `/target_pose`, `/omni_robot/*`. Global planning uses `map`; local control and SLAM bridge use `odom` as usual for this stack.

## Non-obvious pitfalls

- `make stats` wipes `logs/rosbag` and `logs/stats` at start (`make clean`). Copy bags before re-running if you need to keep them.
- Parallel `docker compose run` uses a unique `COMPOSE_PROJECT_NAME` per container to avoid default-network name clashes. After each stats batch, `collect_stats.sh` runs `docker-compose down` per project to remove those networks; `run.sh` runs `down` after an interactive session.
- Headless tmux uses Xvfb so the Gazebo camera keeps publishing usable images for `target_detector`.
- Rosbag folders must contain `metadata.yaml` after a graceful stop; `statistics.py` / `statistics_all.py` skip incomplete dirs.
- `statistics_all` uses `_bag_sort_key`: `run1`…`runN` first (numeric), then other dirs (e.g. `run_YYYYMMDD_…`) by name.

## Quick file pointers

`scripts/run.sh`, `omni_robot/launch/spawn_robot.launch.py`, `omni_robot/launch/omni_nav2.launch.py`, `omni_robot/config/omni_nav2_params.yaml`, `apartment_sim/apartment_sim/obstacles_controller.py`, `nav2_mppi_controller/src/controller.cpp`.
