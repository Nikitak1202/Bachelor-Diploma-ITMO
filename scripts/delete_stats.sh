#!/bin/bash
# Remove all subdirectories under logs/ (e.g. logs/rosbag/*, logs/stats/*).
# Top-level files directly in logs/ are kept.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LOGS_DIR="$PROJECT_ROOT/logs"

if [ ! -d "$LOGS_DIR" ]; then
    echo "Nothing to do: $LOGS_DIR does not exist."
    exit 0
fi

echo "Removing subdirectories under $LOGS_DIR (top-level files preserved)..."
find "$LOGS_DIR" -mindepth 1 -maxdepth 1 -type d -exec rm -rf {} +
