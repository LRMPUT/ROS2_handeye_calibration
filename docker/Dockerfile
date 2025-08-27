# syntax=docker/dockerfile:1
# Select ROS 2 distro at build time: jazzy (Ubuntu 24.04) or humble (Ubuntu 22.04)
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Environment setup
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=${ROS_DISTRO}

# Core tools and Python deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-rosdep \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    bash-completion \
    # ROS packages
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Python libs frequently used by collectors
RUN apt-get update && apt-get install -y --no-install-recommends \ 
    python3-numpy \
    python3-pillow \
    python3-scipy \
    python3-transforms3d \
    && rm -rf /var/lib/apt/lists/*

# Create workspace and clone Robot2Nerf
ENV WS=/ros2_ws
RUN mkdir -p $WS/src
WORKDIR $WS/src
RUN git clone https://github.com/LRMPUT/ROS2_handeye_calibration.git

# Build
WORKDIR $WS
RUN rosdep init 2>/dev/null || true && rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --symlink-install && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source $WS/install/setup.bash" >> /root/.bashrc

#
# Script Setup
#

# Create handeye calibration runner script
RUN cat > /usr/local/bin/run_robot2nerf.sh <<'EOS'
#!/usr/bin/env bash

set -eo pipefail

# Validate input
if [[ "$#" -lt 1 ]]; then
    echo "Usage: $0 <bag_path>"
    exit 1
fi

BAG_PATH="$1"

# Source environments
source /opt/ros/$ROS_DISTRO/setup.bash
source "$WS/install/setup.bash"

# Enable sim time for bag playback
ros2 param set /handeye_calibration use_sim_time true || true

# Cleanup handler for Ctrl+C
cleanup() {
    echo "Shutting down nodes..."
    pkill -P $$ || true
    exit 0
}
trap cleanup EXIT INT TERM

# Run node in background and play bag
ros2 run handeye_calibration handeye_calibration &

# Allow node initialization
sleep 2

# Start bag playback
ros2 bag play "$BAG_PATH" --clock

# Wait for any remaining processes
wait

EOS
RUN chmod +x /usr/local/bin/run_robot2nerf.sh

# Working dir shared via volume
WORKDIR /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]