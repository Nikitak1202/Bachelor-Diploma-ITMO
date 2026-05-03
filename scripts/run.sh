#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOGS_DIR="$PROJECT_ROOT/logs"

# Remove only loose files under logs/ (keep rosbag/, stats/, and other subdirs).
if [ ! -d "$LOGS_DIR" ]; then
    mkdir -p "$LOGS_DIR"
    echo "Created logs directory."
else
    echo "Removing loose files in logs/ (subdirectories kept)..."
    find "$LOGS_DIR" -maxdepth 1 -type f -delete
fi

echo "Stopping any existing apartment_sim container..."
docker-compose down

echo "Removing stale one-off apartment_sim containers..."
STALE_CONTAINERS=$(docker ps -aq --filter "name=apartment_sim-run-")
if [ -n "$STALE_CONTAINERS" ]; then
    docker rm -f $STALE_CONTAINERS >/dev/null
    echo "Removed stale containers:"
    echo "$STALE_CONTAINERS"
else
    echo "No stale one-off containers found."
fi

echo "Building Docker image..."
docker-compose build || exit 1

echo "Starting container with tmux session..."
echo "Detach from tmux with Ctrl+b then d — recording stops, container exits (--rm), then run: make plot"
echo "Open Gazebo in the browser: http://localhost:6080/vnc.html"
echo "Open RViz in the browser: http://localhost:8080/vnc.html"

OMNI_BAG_FINALIZE_SEC="${OMNI_BAG_FINALIZE_SEC:-10}"
docker-compose run --rm -it --service-ports \
    -e OMNI_TMUX_ATTACH=1 \
    -e OMNI_BAG_FINALIZE_SEC="$OMNI_BAG_FINALIZE_SEC" \
    apartment_sim || exit 1
docker-compose down