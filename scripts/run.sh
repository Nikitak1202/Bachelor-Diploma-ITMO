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

# Stop any running container with the same name
echo "Stopping any existing apartment_sim container..."
docker-compose down

# Build the Docker image
echo "Building Docker image..."
docker-compose build

# Run the container interactively (attaches to tmux)
echo "Starting container with tmux session..."
docker-compose run --rm apartment_sim