# ROS2 Hand-Eye Calibration

A ROS2 package for performing hand-eye calibration using a chessboard pattern. The package processes ROS2 bags, with topics from an RGB camera and robot poses, to compute the transformation between the camera and robot end-effector.

## Prerequisites

- Docker
- ROS2 bag containing:
  - RGB images (`/rgb/image_raw`)
  - Camera info (`/rgb/camera_info`)
  - TF transforms between `base_link` and robot end-effector

## Quick Start

1. Create a folder for your calibration bags:
   ```bash
   mkdir bag_folder
   ```

2. Copy your ROS2 bag files into the folder:
   ```bash
   cp /path/to/your/rosbag/* bag_folder/
   ```

3. Run the calibration:
   ```bash
   cd docker
   ./run_docker.sh <your_bag_name>
   ```

4. Find results in the `hand_eye_calibration_data` folder:
   - `hand_eye_calibration_results.json`: Contains the calibration matrix and transformation

## Default Parameters

- Image topic: `/rgb/image_raw`
- Camera info topic: `/rgb/camera_info`
- Source frame: `base_link`
- Target frame: `right_arm_tool0`
- Chessboard pattern: 7x4 internal corners
- Square size: 65mm

## Output

The calibration results include:
- Rotation matrix
- Translation vector
- Quaternion representation

