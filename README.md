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

3. Run the calibration. You have two options:

   Simple usage (with default parameters):
   ```bash
   cd docker
   ./run_docker.sh <your_bag_name>
   ```

   Full usage (customize all parameters):
   ```bash
   cd docker
   ./run_docker.sh <your_bag_name> \
       --image_topic /rgb/image_raw \
       --camera_info_topic /rgb/camera_info \
       --source_frame base_link \
       --target_frame right_arm_tool0 \
       --output_dir hand_eye_calibration_data \
       --collection_rate 20 \
       --chessboard_rows 4 \
       --chessboard_cols 7 \
       --square_size 0.065
   ```

4. Find results in the output directory (default: `hand_eye_calibration_data`):
   - `hand_eye_calibration_results.json`: Contains the calibration matrix and transformation

## Parameters

All parameters are optional and have default values:

- image_topic — (default: `/rgb/image_raw`) RGB camera image topic
- camera_info_topic — (default: `/rgb/camera_info`) RGB camera info topic
- source_frame — (default: `base_link`) Robot base frame
- target_frame — (default: `right_arm_tool0`) Robot end-effector frame
- output_dir — (default: `hand_eye_calibration_data`) Directory for calibration results
- collection_rate — (default: `10`) Collect every Nth frame
- chessboard_rows — (default: `4`) Number of internal corners per row
- chessboard_cols — (default: `7`) Number of internal corners per column
- square_size — (default: `0.065`) Chessboard square size in meters

## Output

The calibration results include:
- Rotation matrix
- Translation vector
- Quaternion representation

