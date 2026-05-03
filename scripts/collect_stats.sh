#!/usr/bin/env bash
# Batched parallel simulation runs (headless tmux), then aggregate plots.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

SIMS="${COLLECT_STATS_SIMS:-3}"
PARALLEL="${COLLECT_STATS_PARALLEL:-2}"
DURATION_SEC="${COLLECT_STATS_DURATION_SEC:-120}"
BAG_FINALIZE_SEC="${COLLECT_STATS_BAG_FINALIZE_SEC:-10}"
DOCKER_STOP_TIMEOUT_SEC="${COLLECT_STATS_DOCKER_STOP_TIMEOUT_SEC:-30}"

HEADLESS_CONFIG="${COLLECT_STATS_TMUX_CONFIG:-/scripts/tmux-cfg-headless.yml}"

ACTIVE_CONTAINERS=()
COMPOSE_PIDS=()
COMPOSE_PROJECT_NAMES=()

compose_down_projects() {
  local proj
  for proj in "${COMPOSE_PROJECT_NAMES[@]}"; do
    [ -n "${proj:-}" ] || continue
    echo "Compose down (remove network): ${proj}" >&2
    (cd "$PROJECT_ROOT" && COMPOSE_PROJECT_NAME="$proj" docker-compose down) || true
  done
}

if ! [[ "$SIMS" =~ ^[0-9]+$ ]] || [ "$SIMS" -lt 1 ]; then
  echo "COLLECT_STATS_SIMS must be a positive integer." >&2
  exit 1
fi
if ! [[ "$PARALLEL" =~ ^[0-9]+$ ]] || [ "$PARALLEL" -lt 1 ]; then
  echo "COLLECT_STATS_PARALLEL must be a positive integer." >&2
  exit 1
fi
if ! [[ "$DURATION_SEC" =~ ^[0-9]+$ ]] || [ "$DURATION_SEC" -lt 1 ]; then
  echo "COLLECT_STATS_DURATION_SEC must be a positive integer." >&2
  exit 1
fi

finalize_bag_in_container() {
  local ctr=$1
  docker exec "$ctr" bash -c '
    set +e
    . /opt/ros/humble/setup.bash
    [ -f /ros2_ws/install/setup.bash ] && . /ros2_ws/install/setup.bash
    tmux send-keys -t apartment_sim:bag_record.0 C-c 2>/dev/null \
      || tmux send-keys -t apartment_sim:bag_record C-c 2>/dev/null || true
    sleep 1
    pkill -INT -f "ros2 bag record" 2>/dev/null || true
    exit 0
  ' 2>/dev/null || true
}

stop_and_remove_container() {
  local ctr=$1
  docker stop -t "$DOCKER_STOP_TIMEOUT_SEC" "$ctr" >/dev/null 2>&1 || true
  docker rm -f "$ctr" >/dev/null 2>&1 || true
}

cleanup_collect_stats() {
  set +e
  set +u
  local c pid
  for c in "${ACTIVE_CONTAINERS[@]}"; do
    [ -n "$c" ] || continue
    echo "Cleaning up container ${c} ..." >&2
    finalize_bag_in_container "$c"
    sleep 1
    stop_and_remove_container "$c"
  done
  for pid in "${COMPOSE_PIDS[@]}"; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  compose_down_projects
  ACTIVE_CONTAINERS=()
  COMPOSE_PIDS=()
  COMPOSE_PROJECT_NAMES=()
  set -e
  set -u
}

trap 'cleanup_collect_stats; exit 130' INT
trap 'cleanup_collect_stats; exit 143' TERM
trap 'cleanup_collect_stats' EXIT

echo "=== Cleaning logs (make clean) ==="
make -C "$PROJECT_ROOT" clean

LOGS_DIR="$PROJECT_ROOT/logs"
mkdir -p "$LOGS_DIR/rosbag"

echo "Stopping any existing apartment_sim container..."
docker-compose down

echo "Removing stale one-off containers..."
STALE_CONTAINERS=$(docker ps -aq --filter "name=apartment_sim-run-")
if [ -n "$STALE_CONTAINERS" ]; then
  docker rm -f $STALE_CONTAINERS >/dev/null
fi
STALE_COLLECT=$(docker ps -aq --filter "name=apartment_collect")
if [ -n "$STALE_COLLECT" ]; then
  docker rm -f $STALE_COLLECT >/dev/null
fi

echo "Building Docker image..."
docker-compose build

for ((batch_start = 1; batch_start <= SIMS; batch_start += PARALLEL)); do
  batch_end=$((batch_start + PARALLEL - 1))
  if [ "$batch_end" -gt "$SIMS" ]; then
    batch_end=$SIMS
  fi

  echo ""
  echo "=== Batch: runs ${batch_start}–${batch_end} / ${SIMS} (${PARALLEL} parallel max, ${DURATION_SEC}s wall-clock) ==="

  docker-compose down
  STALE_CONTAINERS=$(docker ps -aq --filter "name=apartment_sim-run-")
  if [ -n "$STALE_CONTAINERS" ]; then
    docker rm -f $STALE_CONTAINERS >/dev/null
  fi
  STALE_COLLECT=$(docker ps -aq --filter "name=apartment_collect")
  if [ -n "$STALE_COLLECT" ]; then
    docker rm -f $STALE_COLLECT >/dev/null
  fi

  ACTIVE_CONTAINERS=()
  COMPOSE_PIDS=()
  COMPOSE_PROJECT_NAMES=()

  for ((i = batch_start; i <= batch_end; i++)); do
    CTR_NAME="apartment_collect_${i}_$(date +%s)_${RANDOM}_$$"
    RUN_LABEL="run${i}"
    PROJ_NAME="csrun_${i}_${RANDOM}"
    COMPOSE_PROJECT_NAMES+=("$PROJ_NAME")
    echo "Starting ${CTR_NAME} (rosbag ${RUN_LABEL})"
    # Unique project per run avoids parallel "network already exists" races.
    COMPOSE_PROJECT_NAME="$PROJ_NAME" \
      docker-compose run --rm \
      --name "$CTR_NAME" \
      -e OMNI_TMUXP_CONFIG="$HEADLESS_CONFIG" \
      -e OMNI_BAG_RUN_NAME="$RUN_LABEL" \
      -e OMNI_BAG_RUN_SUFFIX="_${RUN_LABEL}" \
      apartment_sim &
    COMPOSE_PIDS+=($!)
    ACTIVE_CONTAINERS+=("$CTR_NAME")
  done

  sleep "$DURATION_SEC"

  for c in "${ACTIVE_CONTAINERS[@]}"; do
    echo "Finalizing rosbag in ${c} ..."
    finalize_bag_in_container "$c"
  done
  echo "Waiting ${BAG_FINALIZE_SEC}s for bag metadata flush ..."
  sleep "$BAG_FINALIZE_SEC"

  for c in "${ACTIVE_CONTAINERS[@]}"; do
    echo "Stopping ${c} ..."
    stop_and_remove_container "$c"
  done

  for pid in "${COMPOSE_PIDS[@]}"; do
    wait "$pid" || true
  done

  compose_down_projects
  ACTIVE_CONTAINERS=()
  COMPOSE_PIDS=()
  COMPOSE_PROJECT_NAMES=()
done

# Portable reset (avoid `trap - SIGNAL`, which some shells treat as "run -").
trap '' EXIT INT TERM

echo ""
echo "=== Aggregate plots (plot-all) ==="
make -C "$PROJECT_ROOT" plot-all
echo ""
echo "=== Latest bag plots (plot) ==="
make -C "$PROJECT_ROOT" plot
