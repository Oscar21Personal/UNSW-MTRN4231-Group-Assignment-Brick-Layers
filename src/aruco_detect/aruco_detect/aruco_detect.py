#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import tf2_ros
from scipy.spatial.transform import Rotation as R
import pyrealsense2 as rs

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        print(cv2.__version__)

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Subscribe to the RGB and depth camera topics
        self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)

        # Publisher for marker information
        self.marker_pub = self.create_publisher(String, '/aruco_markers', 10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ArUco dictionary and parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Depth camera variables
        self.intrinsics = None
        self.depth_image = None

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

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error in depth_callback: {str(e)}")

    def image_callback(self, msg):
        try:
            # Convert the image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = "bgr8")
        except Exception as e:
            self.get_logger().error("CvBridge error: {0}".format(e))
            return

        # Detect markers using the ArucoDetector instance
        corners, ids, _ = self.aruco_detector.detectMarkers(cv_image)

        if ids is not None:
            for i, corner in enumerate(corners):
                marker_id = ids[i][0]

                # Calculate the center of the marker
                marker_center = np.mean(corner[0], axis=0).astype(int)
                center_x, center_y = marker_center[0], marker_center[1]

                # Get depth at the marker's center
                # depth = self.get_depth(center_x, center_y)
                depth = 0.93

                if depth is not None:
                    # Publish transformation based on depth
                    self.publish_transform(corner, marker_id, depth)

                    # Publish marker information
                    marker_info = f"Detected marker ID: {marker_id}, Center: ({center_x}, {center_y}), Depth: {depth:.2f}m"
                    self.marker_pub.publish(String(data=marker_info))

                # Draw the markers on the image
                aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)  # Draw a green circle at the center

        # Display the image
        cv2.imshow("ArUco Detection", cv_image)
        cv2.waitKey(1)

    def get_depth(self, x, y):
        if self.depth_image is not None and self.intrinsics is not None:
            depth_value = self.depth_image[y, x] * 0.001  # Convert mm to meters
            if depth_value > 0:
                return depth_value
        return None

    def publish_transform(self, corners, marker_id, depth):
        # Calculate the center of the marker
        marker_center = np.mean(corners[0], axis=0)
        # marker_center = [int(marker_center[0]), int(marker_center[1])]
        # print("marker_center_type", type(marker_center[0]))
        # Create a TransformStamped message
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = f"marker_{marker_id}"

        # Deproject the pixel to 3D world coordinates
        # print("marker------------------------------------------------", int(marker_center[0]), int(marker_center[1]))
        [x, y, z] = rs.rs2_deproject_pixel_to_point(self.intrinsics, [int(marker_center[0]), int(marker_center[1])], depth)

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z - 0.1

        rotation = R.from_euler('xyz', [0, 180, 0], degrees=True)
        quat = rotation.as_quat()

        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # Broadcast the transform
        # self.get_logger().info(f"marker {marker_id} frame published successfully")
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArucoDetector()

    try:
        rclpy.spin(aruco_detector)
    except KeyboardInterrupt:
        pass
    finally:
        aruco_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
