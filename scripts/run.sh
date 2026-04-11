#!/bin/bash
# Script to start the simulation: cleans logs, stops old containers, builds, and runs

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

echo "Building Docker image..."
docker-compose build || exit 1

echo "Starting container with tmux session..."
echo "Open Gazebo in the browser: http://localhost:6080/vnc.html"
docker-compose run --rm --service-ports apartment_sim || exit 1