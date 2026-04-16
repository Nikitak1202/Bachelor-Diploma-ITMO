#!/bin/bash
LOGS_DIR="./logs"

# Create or clean logs directory
if [ ! -d "$LOGS_DIR" ]; then
    mkdir -p "$LOGS_DIR"
    echo "Created logs directory."
else
    echo "Cleaning logs directory..."
    rm -rf "$LOGS_DIR"/*
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
echo "Open Gazebo in the browser: http://localhost:6080/vnc.html"
docker-compose run --rm --service-ports apartment_sim || exit 1