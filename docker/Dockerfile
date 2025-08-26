# syntax=docker/dockerfile:1
# Select ROS 2 distro at build time: jazzy (Ubuntu 24.04) or humble (Ubuntu 22.04)
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=${ROS_DISTRO}

# Core tools and Python deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-rosdep \
    ros-${ROS_DISTRO}-rosbag2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

# Python libs frequently used by collectors
RUN pip install --no-cache-dir numpy==1.26.* pillow==10.* scipy==1.15.* matplotlib==3.7.*

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

# Convenience runner: plays a bag and runs the collector with params.
# Usage inside container:
#   run_robot2nerf.sh /bags/rosbag2_xxx  \ 
#       --image_topic /rgb/image_raw --camera_info_topic /rgb/camera_info \
#       --source_frame base_link --target_frame azure_rgb \
#       --output_dir /workspace/nerf_dataset/from_bag --collection_rate 10.0
RUN cat > /usr/local/bin/run_robot2nerf.sh <<'EOS'
#!/usr/bin/env bash

set -eo pipefail

if [[ "$#" -lt 1 ]]; then
    echo "Usage: $0 <bag_path> [--image_topic ...] [--camera_info_topic ...] [--source_frame ...] [--target_frame ...] [--output_dir ...] [--collection_rate ...]"
    exit 1
fi

BAG_PATH="$1"; shift || true

# Source environments
source /opt/ros/$ROS_DISTRO/setup.bash
source "$WS/install/setup.bash"

# Defaults can be overridden via flags below
IMAGE_TOPIC="/rgb/image_raw"
CAM_INFO_TOPIC="/rgb/camera_info"
SOURCE_FRAME="base_link"
TARGET_FRAME="azure_rgb"
OUTPUT_DIR="/workspace/nerf_dataset/dataset_from_bag"
RATE="10.0"

# Parse simple --key value flags
while (( "$#" )); do
    case "$1" in
        --image_topic) IMAGE_TOPIC="$2"; shift 2;;
        --camera_info_topic) CAM_INFO_TOPIC="$2"; shift 2;;
        --source_frame) SOURCE_FRAME="$2"; shift 2;;
        --target_frame) TARGET_FRAME="$2"; shift 2;;
        --output_dir) OUTPUT_DIR="$2"; shift 2;;
        --collection_rate) RATE="$2"; shift 2;;
        *) echo "Unknown arg: $1"; exit 2;;
    esac
done

# Cleanup handler for Ctrl+C
cleanup() {
    echo "Shutting down nodes..."
    pkill -P $$ || true
    exit 0
}
trap cleanup EXIT INT TERM

# Start your node first
ros2 run handeye_calibration handeye_calibration \
    --ros-args \
    -p image_topic:=${IMAGE_TOPIC} \
    -p camera_info_topic:=${CAM_INFO_TOPIC} \
    -p source_frame:=${SOURCE_FRAME} \
    -p target_frame:=${TARGET_FRAME} \
    -p output_dir:=${OUTPUT_DIR} \
    -p collection_rate:=${RATE} \
    -p use_sim_time:=true &

# Give the node a moment to start
sleep 1

# Now play the bag - this will block until playback finishes
# or until you press Ctrl+C
ros2 bag play "$BAG_PATH"

echo "Bag playback complete. Finalizing collection..."

EOS
RUN chmod +x /usr/local/bin/run_robot2nerf.sh

# Working dir shared via volume
WORKDIR /workspace

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]