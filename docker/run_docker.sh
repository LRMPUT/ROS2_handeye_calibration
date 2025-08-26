#!/usr/bin/env bash
# Simple runner for the ROS2 collector container.
# Usage:
#   ./run_docker.sh bag_folder/tomato_soup_can \
#     --image_topic /rgb/image_raw \
#     --camera_info_topic /rgb/camera_info \
#     --source_frame base_link \
#     --target_frame azure_rgb \
#     --output_dir /workspace/outputs/dataset_from_bag \
#     --collection_rate 10

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ROS distro from env or default to humble
DISTRO="${ROS_DISTRO:-humble}"
IMAGE_NAME="ros2_collector:${DISTRO}"

if [[ "$#" -lt 1 ]]; then
	echo "Usage: $0 <repo-relative-bag-dir> [collector args]"
	exit 1
fi

BAG_DIR_REL="$1"; shift || true
COLLECTOR_ARGS=( "$@" )

if [[ ! -d "$REPO_ROOT/$BAG_DIR_REL" ]]; then
	echo "Error: '$BAG_DIR_REL' not found under repo: $REPO_ROOT"
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
