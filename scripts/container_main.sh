#!/usr/bin/env bash
# Default container process: load tmuxp (detached) and keep PID 1 alive.
# OMNI_TMUXP_CONFIG is read at runtime (avoids brittle CMD JSON / compose $ escaping).
set -euo pipefail
CONFIG="${OMNI_TMUXP_CONFIG:-/scripts/tmux-cfg.yml}"
if [[ ! -f "$CONFIG" ]]; then
  echo "TMUXP config not found: $CONFIG" >&2
  exit 1
fi
tmuxp load -d "$CONFIG"

finalize_after_tmux_detach() {
  local wait_sec="${OMNI_BAG_FINALIZE_SEC:-10}"
  # ROS setup.bash references unset vars; must not run under nounset.
  set +eu
  # shellcheck disable=SC1091
  [ -f /opt/ros/humble/setup.bash ] && . /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  [ -f /ros2_ws/install/setup.bash ] && . /ros2_ws/install/setup.bash
  tmux send-keys -t apartment_sim:bag_record.0 C-c 2>/dev/null \
    || tmux send-keys -t apartment_sim:bag_record C-c 2>/dev/null || true
  sleep 1
  pkill -INT -f "ros2 bag record" 2>/dev/null || true
  sleep "$wait_sec"
  tmux kill-server 2>/dev/null || true
}

if [[ "${OMNI_TMUX_ATTACH:-}" == "1" ]] && [ -t 1 ]; then
  tmux attach -t apartment_sim || true
  finalize_after_tmux_detach
  exit 0
fi

exec sleep infinity
