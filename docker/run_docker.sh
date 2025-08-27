#!/usr/bin/env bash
# Simple runner for the ROS2 collector container.
# Usage:
#   ./run_docker.sh calibr_eryk


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

# Construct path relative to bag_folder
BAG_NAME="$1"
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
	"${COLLECTOR_ARGS[@]}"
