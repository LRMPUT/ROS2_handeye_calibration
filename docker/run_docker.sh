#!/usr/bin/env bash
# Simple runner for the ROS2 collector container.
# Usage:
#   ./run_docker.sh calibr_eryk
#
# Or full command
# ./run_docker.sh calibr_eryk \
#     --image_topic /rgb/image_raw \
#     --camera_info_topic /rgb/camera_info \
#     --source_frame base_link \
#     --target_frame right_arm_tool0 \
#     --output_dir hand_eye_calibration_data \
#     --collection_rate 20 \
#     --chessboard_rows 4 \
#     --chessboard_cols 7 \
#     --square_size 0.065



set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ROS distro from env or default to humble
DISTRO="${ROS_DISTRO:-humble}"
IMAGE_NAME="ros2_collector:${DISTRO}"

if [[ "$#" -lt 1 ]]; then
	echo "Usage: $0 <bag_name>"
	echo "Note: Place your bags in the bag_folder directory"
	exit 1
fi

# Get the bag name from first argument
BAG_NAME="$1"
shift

# Initialize ROS parameters array
ROS_ARGS=(--ros-args )

# Parse remaining arguments
while [[ $# -gt 0 ]]; do
    if [[ "$1" =~ ^--(.+)$ ]]; then
        param_name="${BASH_REMATCH[1]}"
        if [[ $# -lt 2 ]]; then
            echo "Error: Missing value for parameter $param_name"
            exit 1
        fi
        ROS_ARGS+=(-p "${param_name}:=$2")
        shift 2
    else
        echo "Error: Parameter names must start with --"
        echo "Usage: $0 <bag_name> [--param_name value]..."
        exit 1
    fi
done

# Construct path relative to bag_folder
BAG_DIR_REL="bag_folder/$BAG_NAME"

# Check if bag folder exists
if [[ ! -d "$REPO_ROOT/$BAG_DIR_REL" ]]; then
    echo "Error: Bag '$BAG_NAME' not found in bag_folder"
    echo "Please place your bag files in: $REPO_ROOT/bag_folder/$BAG_NAME"
    exit 2
fi

# Build image if missing
if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
	echo "Building image: $IMAGE_NAME (ROS_DISTRO=$DISTRO)"
	docker build -t "$IMAGE_NAME" --build-arg ROS_DISTRO="$DISTRO" -f "$SCRIPT_DIR/Dockerfile" "$SCRIPT_DIR"
fi

# Mount the entire repo at /workspace and run the collector
docker run --rm \
    -e ROS_DISTRO="$DISTRO" \
    -v "$REPO_ROOT":/workspace \
    "$IMAGE_NAME" \
    run_robot2nerf.sh \
    "/workspace/$BAG_DIR_REL" \
    "${ROS_ARGS[@]}"
