#!/usr/bin/env python3

import os
import cv2
import numpy as np
import json
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformListener, Buffer
from cv_bridge import CvBridge


class HandEyeCalibration(Node):
    """
    Simplified ROS2 Node that collects images and camera poses for hand-eye calibration.
    """
    
    def __init__(self):
        super().__init__('hand_eye_calibration')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/rgb/camera_info')
        self.declare_parameter('source_frame', 'base_link')
        self.declare_parameter('target_frame', 'right_arm_tool0')
        self.declare_parameter('output_dir', 'hand_eye_calibration_data')
        self.declare_parameter('collection_rate', 10)
        self.declare_parameter('chessboard_rows', 4)  # internal corners
        self.declare_parameter('chessboard_cols', 7)  # internal corners
        self.declare_parameter('square_size', 0.065)  # 65mm

        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.source_frame = self.get_parameter('source_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.output_dir = self.get_parameter('output_dir').value
        self.collection_rate = self.get_parameter('collection_rate').value
        self.chessboard_rows = self.get_parameter('chessboard_rows').value
        self.chessboard_cols = self.get_parameter('chessboard_cols').value
        self.square_size = self.get_parameter('square_size').value

        # Initialize
        self.bridge = CvBridge()
        self.camera_params = None
        self.frames_data = []
        self.frame_count = 0
        self.i = 0
        self.collecting = False
        self.start_timer_called = False
        self.latest_image = None
        self.latest_depth = None
        self.K = None
        self.dist_coeffs = None
        self.images = []
        
        # Create 3D object points for chessboard
        self.objp = np.zeros((self.chessboard_rows * self.chessboard_cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.chessboard_cols, 0:self.chessboard_rows].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Storage for calibration data
        self.gripper_translations = []
        self.gripper_rotations = []
        self.board_translations = []
        self.board_rotations = []

        # Timer for detecting image topic silence
        self.last_image_time = None
        self.silence_timer = None
        self.silence_duration = 3.0  # 3 seconds
        self.collection_finished = False

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, 1
        )
        
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 1
        )
        
        # Print all parameters
        self.get_logger().info("Hand-Eye Calibration initialized with parameters:")
        self.get_logger().info(f"  Image topic: {self.image_topic}")
        self.get_logger().info(f"  Camera info topic: {self.camera_info_topic}")
        self.get_logger().info(f"  Source frame: {self.source_frame}")
        self.get_logger().info(f"  Target frame: {self.target_frame}")
        self.get_logger().info(f"  Output directory: {self.output_dir}")
        self.get_logger().info(f"  Collection rate: {self.collection_rate}")
        self.get_logger().info(f"  Chessboard size: {self.chessboard_cols}x{self.chessboard_rows}")
        self.get_logger().info(f"  Square size: {self.square_size}m")

    def image_callback(self, msg):
        """Store the latest image."""
        if not self.collecting:
            self.get_logger().warn("Not collecting yet, waiting for camera info")
            return
        self.last_image_time = self.get_clock().now()
        self.reset_silence_timer()

        if self.i % self.collection_rate == self.collection_rate - 1:  # Collect every nth image
            self.latest_image = msg
            self.collect_frame()

        self.i += 1
    def reset_silence_timer(self):
        """Reset the silence detection timer."""
        if self.silence_timer is not None:
            self.silence_timer.cancel()

        self.silence_timer = self.create_timer(self.silence_duration, self.check_silence)

    def check_silence(self):
        """Check if image topic has been silent for too long."""
        if self.collection_finished:
            return
        
        current_time = self.get_clock().now()
        if self.last_image_time is not None:
            elapsed = (current_time - self.last_image_time).nanoseconds / 1e9

            if elapsed >= self.silence_duration:
                self.get_logger().info("Image topic silent for too long, finishing collection")
                self.finish_collection()
                self.collection_finished = True

    def camera_info_callback(self, msg):
        """Get camera parameters once."""
        if self.K is None or self.dist_coeffs is None:

            self.K = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            
            self.get_logger().info("Camera parameters received")
            self.collecting = True
            self.get_logger().info("Starting collection")
    
    def collect_frame(self):
        """Collect one frame (image + pose)."""
        
        if self.latest_image is None:
            self.get_logger().warn("No image received yet")
            return
        
        try:
            # Use the latest image
            image_msg = self.latest_image
            
            # Get transform for image timestamp
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                image_msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Save image
            cv_img = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            # cv_img = cv2.resize(cv_img, (640, 480))
            self.images.append(cv_img)
            
            # Store gripper pose
            t = transform.transform
            translation = [t.translation.x, t.translation.y, t.translation.z]
            rotation = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
            
            self.gripper_translations.append(translation)
            self.gripper_rotations.append(rotation)

            # Convert transform to NeRF format
            T = self.to_matrix(transform)
            self.frames_data.append(T)

            self.frame_count += 1
            self.get_logger().info(f"Collected frame {self.frame_count}")
            
        except Exception as e:
            self.get_logger().warn(f"Failed to collect frame: {str(e)}")
    
    def normalize_rotation_matrices(self, mats):
        """Properly normalize rotation matrices to ensure they are valid rotation matrices"""
        fixed_mats = []
        for mat in mats:
            mat = np.array(mat)
            
            # Ensure the matrix is orthogonal and has determinant +1
            u, s, vh = np.linalg.svd(mat)
            
            # Construct proper rotation matrix
            R = np.dot(u, vh)
            
            # Ensure determinant is +1 (not -1 which would be a reflection)
            if np.linalg.det(R) < 0:
                vh[-1, :] *= -1
                R = np.dot(u, vh)
            
            fixed_mats.append(R)
        
        return fixed_mats

    def to_matrix(self, tf_transform):
        """Convert ROS transform to NeRF Studio format."""
        # Extract pose from ROS transform
        t = tf_transform.transform
        translation = np.array([t.translation.x, t.translation.y, t.translation.z])
        rotation = R.from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        
        # Create 4x4 homogeneous matrix (camera-to-world)
        T_ros = np.eye(4)
        rot_matrix = rotation.as_matrix()
        # Normalize the rotation matrix to ensure orthogonality
        u, _, vh = np.linalg.svd(rot_matrix)
        rot_matrix_normalized = np.dot(u, vh)
        T_ros[:3, :3] = rot_matrix_normalized
        T_ros[:3, 3] = translation

        # Apply coordinate transformation
        T_nerf = T_ros 
        
        return T_nerf  # Return full 4x4 matrix
    
    def image_processing(self, i, image):
        """Process an image to detect chessboard and extract pose."""
        if image is None:
            print(f"Could not load image {i}")
            return False
            
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, (self.chessboard_cols, self.chessboard_rows), None)   

        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)

            cv2.drawChessboardCorners(image, (self.chessboard_cols, self.chessboard_rows), corners, ret)
            success, rvec, tvec = cv2.solvePnP(self.objp, corners, self.K, self.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
            
            if not success:
                print(f"solvePnP failed for image {i}")
                return False
                
            R_matrix, _ = cv2.Rodrigues(rvec)
            
            # Validate the pose - check if translation is reasonable
            tvec_norm = np.linalg.norm(tvec)
            if tvec_norm > 5.0 or tvec_norm < 0.01:  # Sanity check: between 1cm and 5m
                print(f"Unreasonable translation magnitude {tvec_norm:.3f} for image {i}")
                return False

            # Define 3D points for drawing axes (X: red, Y: green, Z: blue)
            axis_length = 0.05  # length of the axes in meters
            axes_3d = np.float32([
                [0, 0, 0],                # origin
                [axis_length, 0, 0],      # X axis
                [0, axis_length, 0],      # Y axis
                [0, 0, -axis_length]      # Z axis (negative for proper visualization)
            ])
            
            # Project 3D points to image plane
            imgpts, _ = cv2.projectPoints(axes_3d, rvec, tvec, self.K, self.dist_coeffs)
            imgpts = np.int32(imgpts).reshape(-1, 2)

            # Draw axes on image
            corner = tuple(imgpts[0])  # origin
            cv2.line(image, corner, tuple(imgpts[1]), (0, 0, 255), 3)  # X axis in red
            cv2.line(image, corner, tuple(imgpts[2]), (0, 255, 0), 3)  # Y axis in green
            cv2.line(image, corner, tuple(imgpts[3]), (255, 0, 0), 3)  # Z axis in blue 

            # Save processed image for debugging (optional)
            # debug_path = file_path.replace('.png', '_debug.png')
            # cv2.imwrite(debug_path, image)

            self.board_translations.append(tvec.flatten())
            self.board_rotations.append(R_matrix.flatten())
            return True
        else:
            # print(f"No chessboard corners found in {file_path}") 
            return False
    
    def finish_collection(self):
        """Process collected images and perform hand-eye calibration."""
        if self.collection_finished:
            return
        
        self.collection_finished = True
        self.collecting = False
        
        if self.silence_timer is not None:
            self.silence_timer.cancel()

        if self.frame_count == 0:
            self.get_logger().error("No frames collected!")
            return
        
        # Process collected images to detect chessboards
        valid_indices = []
        for i, image in enumerate(self.images):
            found = self.image_processing(i, image)
            if found:
                valid_indices.append(i)

        print(f"Found {len(valid_indices)} valid chessboard detections out of {len(self.images)} images")

        # Filter gripper poses based on valid chessboard detections
        gripper_translations = [self.gripper_translations[i] for i in valid_indices]
        gripper_rotations = [self.gripper_rotations[i] for i in valid_indices]

        if len(gripper_translations) < 3:
            print("Error: Need at least 3 valid pose pairs for calibration")
            return

        R_gripper2base = [R.from_quat(q).as_matrix() for q in gripper_rotations]
        t_gripper2base = [np.array(t).reshape(3, 1) for t in gripper_translations]

        R_target2cam = [r.reshape(3, 3) for r in self.board_rotations]
        t_target2cam = [np.array(t).reshape(3, 1) for t in self.board_translations]

        print(f"Before normalization - checking rotation matrices...")
        for i, (R_g, R_t) in enumerate(zip(R_gripper2base[:3], R_target2cam[:3])):
            print(f"Gripper {i}: det={np.linalg.det(R_g):.6f}")
            print(f"Target {i}: det={np.linalg.det(R_t):.6f}")

        R_gripper2base = self.normalize_rotation_matrices(R_gripper2base)
        R_target2cam = self.normalize_rotation_matrices(R_target2cam)

        print(f"After normalization - checking rotation matrices...")
        for i, (R_g, R_t) in enumerate(zip(R_gripper2base[:3], R_target2cam[:3])):
            print(f"Gripper {i}: det={np.linalg.det(R_g):.6f}")
            print(f"Target {i}: det={np.linalg.det(R_t):.6f}")

        # OpenCV Hand-Eye Calibration
        print("Running OpenCV hand-eye calibration...")
        r_cam2grip, t_cam2grip = cv2.calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=cv2.CALIB_HAND_EYE_PARK
        )
        
        rotation = R.from_matrix(r_cam2grip)
        quat = rotation.as_quat()

        print(f"OpenCV result:")
        print(f"r_cam2grip quaternion: {quat}")
        print(f"t_cam2grip: {t_cam2grip.flatten()}")

        flat_t_cam2grip = [str(item[0]) for item in t_cam2grip]
        print(f"ROS2 command: ros2 run tf2_ros static_transform_publisher {' '.join(map(str, flat_t_cam2grip))} {' '.join(map(str, quat))} gripper_right_base_link camera_frame")

        # Simple visualization
        print(f"Final transformation matrix:")
        T_c2g_final = np.eye(4)
        T_c2g_final[:3, :3] = r_cam2grip
        T_c2g_final[:3, 3] = t_cam2grip.flatten()
        print(T_c2g_final)

        # Save calibration results
        self.save_calibration_results(r_cam2grip, t_cam2grip)

        self.get_logger().info(f"Collection complete! Processed {len(valid_indices)} frames")
        
        # Shutdown
        rclpy.shutdown()
    
    def save_calibration_results(self, r_cam2grip, t_cam2grip):
        """Save calibration results to file."""
        results = {
            'rotation_matrix': r_cam2grip.tolist(),
            'translation_vector': t_cam2grip.flatten().tolist(),
            'quaternion': R.from_matrix(r_cam2grip).as_quat().tolist(),
            'chessboard_size': [self.chessboard_cols, self.chessboard_rows],
            'square_size': self.square_size,
            'num_valid_poses': len(self.board_translations)
        }
        
        os.makedirs(self.output_dir, exist_ok=True)
        result_file = os.path.join(self.output_dir, 'hand_eye_calibration_results.json')
        
        with open(result_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"Results saved to {result_file}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        collector = HandEyeCalibration()
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.finish_collection()
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()