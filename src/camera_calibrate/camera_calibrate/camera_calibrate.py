#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class CameraCalibrate(Node):
    def __init__(self):
        super().__init__('camera_calibrate')

        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()

        self.xval = 0.4
        self.yval = 0.04
        self.zval = 0.001

        # Known 3D positions of ArUco markers in the robot base frame (in meters)
        self.marker_positions = {
            1: np.array([self.xval, self.yval, self.zval]),
            2: np.array([self.xval + 0.07, self.yval, self.zval]),
            3: np.array([self.xval + 0.14, self.yval, self.zval]),
            4: np.array([self.xval, self.yval + 0.07, self.zval]),
            5: np.array([self.xval + 0.07, self.yval + 0.07, self.zval]),
            6: np.array([self.xval + 0.14, self.yval + 0.07, self.zval]),
        }

        # Subscribe to camera topics
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)
        self.d_cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.d_camera_info_callback, 10)

        # Camera Tf static publisher
        self.camera_tf_broadcaster = StaticTransformBroadcaster(self)

        # ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Depth camera intrinsics
        self.intrinsics = None
        self.d_intrinsics = None
        self.depth_image = None

        # Camera values
        self.camera_pos = []
        self.camera_rot = []

        self.posted_tf = False

    def camera_info_callback(self, camera_info):
        try:
            if self.intrinsics is None:
                self.intrinsics = rs.intrinsics()
                self.intrinsics.width = camera_info.width
                self.intrinsics.height = camera_info.height
                self.intrinsics.ppx = camera_info.k[2]
                self.intrinsics.ppy = camera_info.k[5]
                self.intrinsics.fx = camera_info.k[0]
                self.intrinsics.fy = camera_info.k[4]
                self.intrinsics.coeffs = [i for i in camera_info.d]
                if camera_info.distortion_model == 'plumb_bob':
                    self.intrinsics.model = rs.distortion.brown_conrady
                elif camera_info.distortion_model == 'equidistant':
                    self.intrinsics.model = rs.distortion.kannala_brandt4
        except CvBridgeError as e:
            print(e)
            return

    def d_camera_info_callback(self, camera_info):
        try:
            if self.d_intrinsics is None:
                self.d_intrinsics = rs.intrinsics()
                self.d_intrinsics.width = camera_info.width
                self.d_intrinsics.height = camera_info.height
                self.d_intrinsics.ppx = camera_info.k[2]
                self.d_intrinsics.ppy = camera_info.k[5]
                self.d_intrinsics.fx = camera_info.k[0]
                self.d_intrinsics.fy = camera_info.k[4]
                self.d_intrinsics.coeffs = [i for i in camera_info.d]
                if camera_info.distortion_model == 'plumb_bob':
                    self.d_intrinsics.model = rs.distortion.brown_conrady
                elif camera_info.distortion_model == 'equidistant':
                    self.d_intrinsics.model = rs.distortion.kannala_brandt4
        except CvBridgeError as e:
            print(e)
            return

    def depth_callback(self, msg):
        """Callback to store the latest depth image."""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error in depth_callback: {str(e)}")

    def pixel_to_global(self, pixel):
        # Convert 2D image pixel to 3D world coordinates
        #pixel = [int(pixel[0]), int(pixel[1])]

        if self.depth_image is not None and self.d_intrinsics is not None:
            
            depth = self.depth_image[int(pixel[1]), int(pixel[0])] * 0.001  # Convert depth to meters
            print(pixel,depth)
            return rs.rs2_deproject_pixel_to_point(self.d_intrinsics, [pixel[0], pixel[1]], depth)
        return rs.rs2_deproject_pixel_to_point(self.d_intrinsics, [pixel[0], pixel[1]], 0.9388)

    def image_callback(self, msg):
        """Callback to detect markersose relative to the robot base frame
            # self.get_logger().info(f"Camera Position (x, y, z): {tvec.ravel()}")
            # self.get_logger().info(f and estimate camera pose."""
        try:
            # Convert ROS image message to OpenCV image
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")
            return

        # Detect markers using the ArucoDetector instance
        corners, ids, _ = self.aruco_detector.detectMarkers(color_image)

        if ids is not None:
            # Initialize a dictionary to store detected markers with IDs as keys
            detected_markers = {}

            # Populate the dictionary with marker ID as key and corners as value
            for i, marker_id in enumerate(ids.flatten()):
                detected_markers[int(marker_id)] = corners[i]

            # Sort detected markers by marker ID
            sorted_markers = sorted(detected_markers.items(), key=lambda item: item[0])

            # Initialize lists for sorted marker corners and known 3D points
            marker_corners = []
            marker_points_3D = []

            # Match sorted marker IDs with known positions
            for marker_id, corner in sorted_markers:
                if marker_id in self.marker_positions:
                    marker_corners.append(corner)
                    marker_points_3D.append(self.marker_positions[marker_id])

            # Proceed if we have enough markers (minimum 4 for solvePnP)
            if len(marker_corners) >= 6:
                self.calculate_camera_pose(marker_points_3D, marker_corners)

            # Draw the markers on the image
            aruco.drawDetectedMarkers(color_image, corners, ids)

            # Plot the center of each detected marker
            for corner in corners:
                center = np.mean(corner[0], axis=0).astype(int)  # Calculate center of the marker
                cv2.circle(color_image, tuple(center), 5, (0, 0, 255), -1)  # Draw a red circle at the center

        # Display the image
        cv2.imshow("ArUco Detection", color_image)
        cv2.waitKey(1)

    def calculate_camera_pose(self, marker_points_3D, marker_corners):
        """Estimate camera pose based on detected marker positions."""
        # Convert list of 3D points to numpy array
        object_points = np.array(marker_points_3D, dtype=np.float32)

        # Calculate 2D image points by taking the center of each marker
        image_points = []
        for corner in marker_corners:
            center = np.mean(corner[0], axis=0)
            image_points.append(center)
        image_points = np.array(image_points, dtype=np.float32)

        # Construct the camera matrix from intrinsics
        camera_matrix = np.array([[self.intrinsics.fx, 0, self.intrinsics.ppx],
                                  [0, self.intrinsics.fy, self.intrinsics.ppy],
                                  [0, 0, 1]])

        # Use the distortion coefficients from the camera
        dist_coeffs = np.array(self.intrinsics.coeffs)

        # Use solvePnP to find the rotation and translation vectors
        success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)
        if success:
            # Convert rotation vector to roll, pitch, yaw
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            rotation_matrix_t = rotation_matrix.T
            rotation_matrix_cam = R.from_euler('xyz',[90,-90,0], degrees=True).as_matrix()
            # print(rotation_matrix_t,rotation_matrix_cam,rotation_matrix_t*rotation_matrix_cam,np.matrix(rotation_matrix_t)*np.matrix(rotation_matrix_cam))

            rpy = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=True)
            quat1 = R.from_matrix(np.matrix(rotation_matrix_cam).T*np.matrix(rotation_matrix_t)).as_quat()

            rpy_t = R.from_quat(quat1).as_euler('xyz', degrees=True)
            quat2 = R.from_euler('xyz',rpy_t, degrees=True).as_quat()

            # # Output the camera pose relative to the robot base frame
            # self.get_logger().info(f"Camera Position (x, y, z): {tvec.ravel()}")
            # self.get_logger().info(f"Camera Orientation (roll, pitch, yaw): {rpy}")

            p = R.from_euler('zyx', [rpy[2], rpy[1], rpy[0]], degrees=True)
            p2 = p.inv()

            self.camera_pos = -np.matrix(rotation_matrix).T * np.matrix(tvec)
            self.camera_rot = rpy
            ts = []
            # if not self.posted_tf:
                # # Output the camera pose relative to the robot base frame
            self.get_logger().info(f"Camera Position (x, y, z): {tvec.ravel()}")
            self.get_logger().info(f"Camera Orientation (roll, pitch, yaw): {rpy}")


            t= TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            # t.child_frame_id = 'camera_color_optical_frame'

            # t.transform.translation.x = float(self.camera_pos[0])
            # t.transform.translation.y = float(self.camera_pos[1])
            # t.transform.translation.z = float(self.camera_pos[2])
            # t.transform.rotation.x = quat1[0]
            # t.transform.rotation.y = quat1[1]
            # t.transform.rotation.z = quat1[2]
            # t.transform.rotation.w = quat1[3]

            # ts.append(t)

            t.child_frame_id = 'camera_link'
            t.transform.translation.x = float(self.camera_pos[0])
            t.transform.translation.y = float(self.camera_pos[1])
            t.transform.translation.z = float(self.camera_pos[2])
            t.transform.rotation.x = quat2[0]
            t.transform.rotation.y = quat2[1]
            t.transform.rotation.z = quat2[2]
            t.transform.rotation.w = quat2[3]

            ts.append(t)
            aruco_count = 1
            print(image_points)
            for pxl in image_points:
                aruco_pos = self.pixel_to_global(pxl)
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_color_optical_frame'
                t.child_frame_id = 'aruco_' + str(aruco_count)

                t.transform.translation.x = aruco_pos[0]
                t.transform.translation.y = aruco_pos[1]
                t.transform.translation.z = aruco_pos[2]
                t.transform.rotation.x = 0.
                t.transform.rotation.y = 0.
                t.transform.rotation.z = 0.
                t.transform.rotation.w = 1.
                ts.append(t)
                aruco_count +=1
                

            self.camera_tf_broadcaster.sendTransform(ts)
            self.posted_tf = True



            # pos = np.array([tvec.ravel()]).T
            # print("loc", pos)
            # print('rot', p.as_matrix())
            # print('reg', np.matmul(p.as_matrix(), pos))

            # print('rot inv', p2.as_matrix())
            # print('inv', np.matmul(p2.as_matrix(), pos))

def main(args=None):
    rclpy.init(args=args)
    camera_calibrate = CameraCalibrate()

    try:
        rclpy.spin(camera_calibrate)
    except KeyboardInterrupt:
        pass
    finally:
        camera_calibrate.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()