import rclpy
import cv2
import tf2_ros
import numpy as np
import pyrealsense2 as rs
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros.transform_broadcaster import TransformBroadcaster
 
 
class ColorMaskNode(Node):
 
    def __init__(self):
        super().__init__('color_mask_node')
 
        # Initialize attributes
        self.cv_image = None
        self.depth_image = None
        self.intrinsics = None
        
        # Bridge for converting ROS Image messages to OpenCV images
        self.bridge = CvBridge()
 
        # Subscriptions to color and depth image topics
        self.image_sub = self.create_subscription(
            Image, 'camera/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.camera_info_callback, 10)
 
        # TF broadcaster for sending the object transform
        self.tf_broadcaster = TransformBroadcaster(self)
 
        # Timer to run the routine callback periodically (20 Hz)
        self.timer = self.create_timer(0.05, self.routine_callback)

        # List of block global coordinates
        self.block_list = []
 
    def camera_info_callback(self, camera_info):
        # Retrieve camera intrinsics from CameraInfo message
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = camera_info.width
            self.intrinsics.height = camera_info.height
            self.intrinsics.ppx = camera_info.k[2]
            self.intrinsics.ppy = camera_info.k[5]
            self.intrinsics.fx = camera_info.k[0]
            self.intrinsics.fy = camera_info.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady if camera_info.distortion_model == 'plumb_bob' else rs.distortion.kannala_brandt4
            self.intrinsics.coeffs = camera_info.d
 
    def depth_callback(self, msg):
        try:
            # Convert the ROS depth image message to OpenCV format
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {str(e)}")
 
    def image_callback(self, msg):
        try:
            # Convert the ROS color image message to OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting color image: {str(e)}")
 
    def pixel_to_global(self, pixel):
        # Convert 2D image pixel to 3D world coordinates
        if self.depth_image is not None and self.intrinsics is not None:
            #depth = self.depth_image[pixel[1], pixel[0]] * 0.001  # Convert depth to meters
            depth = 0.9
            return rs.rs2_deproject_pixel_to_point(self.intrinsics, [pixel[0], pixel[1]], depth)
        return None
 
    def routine_callback(self):
        if self.cv_image is None:
            self.get_logger().warning("No image received, skipping routine.")
            return

        # Convert BGR image to HSV for more accurate color detection
        hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        # Make the image darker
        #hsv_image= cv2.add(hsv_image,np.array([-20.0]))

        # Apply a green color mask to detect the object (adjust the HSV range as needed)
        lower_green = np.array([70, 160, 70])  # Lower bound for green
        upper_green = np.array([85, 235, 110])  # Upper bound for green
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # Closing and opening
        kernal = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal, iterations=5)
        # mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal, iterations=1)
        # mask = cv2.dilate(mask, kernal, iterations=1)

        # Apply Gaussian blur to reduce noise
        mask = cv2.GaussianBlur(mask, (9, 9), 0)

        # Display the refined mask
        cv2.imshow("Refined Green Color Mask", mask)
        cv2.waitKey(1)

        # Find contours of the masked objects
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.block_list.clear()

        # Iterate through contours and find the centers of the objects
        min_area = 500  # Minimum contour area to consider valid (adjust as needed)
        for contour in contours:
            if cv2.contourArea(contour) < min_area:  # Filter small contours
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    # Calculate the center of the object
                    cX = int(moments['m10'] / moments['m00'])
                    cY = int(moments['m01'] / moments['m00'])
                    
                    # Convert the pixel coordinates to 3D world coordinates
                    global_position = self.pixel_to_global([cX, cY])
                    if global_position is not None:
                        # Append the global position to the list
                        self.block_list.append(global_position)

                    # Mark the center on the image
                    cv2.circle(self.cv_image, (cX, cY), 5, (0, 0, 255), -1)  # Red dot at center
                    cv2.putText(self.cv_image, "Center", (cX - 20, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)  # Label

        # Display the image with the marked centers
        cv2.imshow("Detected Objects with Centers", self.cv_image)
        cv2.waitKey(1)

        # Broadcast transforms for all detected blocks
        for idx, block_coords in enumerate(self.block_list):
            self.broadcast_transform(block_coords, idx)

        # Log the global coordinates of all detected blocks
        self.get_logger().info(f"Detected block positions: {self.block_list}")

    def broadcast_transform(self, position, idx):
        transform = TransformStamped()

        # Set the header information
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_color_optical_frame'
        transform.child_frame_id = f'free_block_{idx}'  # Unique name for each block

        # Set the transform translation (adjust for camera offset if needed)
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2] - 0.1

        # Send the transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = ColorMaskNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
 
