# ROS2 Humble base image — amd64
FROM --platform=linux/amd64 ros:humble-ros-base

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-xacro \
    ros-humble-rviz2 \ 
    python3-pip \
    tmux \
    tmuxp \
    xvfb \
    x11vnc \
    novnc \
    git \
    vim \
    && pip3 install --no-cache-dir websockify \
    && rm -rf /var/lib/apt/lists/*

# Clone the apartment world repository (models and world)
RUN git clone https://github.com/aws-robotics/aws-robomaker-small-house-world.git /tmp/aws-robomaker-small-house-world

# Copy all models to Gazebo's user model directory (ensures they are found)
RUN mkdir -p /root/.gazebo/models && \
    cp -r /tmp/aws-robomaker-small-house-world/models/* /root/.gazebo/models/

# Set Gazebo model path to include the cloned models (as fallback)
ENV GAZEBO_MODEL_PATH=/tmp/aws-robomaker-small-house-world/models:$GAZEBO_MODEL_PATH

# Copy the world file to /worlds (will be overridden by mount, but serves as fallback)
RUN mkdir -p /worlds
RUN cp /tmp/aws-robomaker-small-house-world/worlds/small_house.world /worlds/apartment.world

# Create workspace and copy source code
WORKDIR /ros2_ws
COPY src/ ./src/

# Create logs directory
RUN mkdir -p /ros2_ws/logs

# Build the ROS2 package
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Copy scripts and entrypoint
COPY scripts/ /scripts/
COPY scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /scripts/*.sh

RUN apt-get update && apt-get install -y \
    openbox \
    xdotool \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/entrypoint.sh"]
CMD ["tmuxp", "load", "/scripts/tmux-cfg.yml"]