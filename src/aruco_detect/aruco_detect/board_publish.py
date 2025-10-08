#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

class BoardPublisher(Node):
    def __init__(self):
        super().__init__('board_publisher')

        # Initialize tf buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize tf broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Set timer to publish the board frame periodically
        self.timer = self.create_timer(0.1, self.publish_board_frame)  # 10 Hz

    def get_transform(self, parent_frame, child_frame):
        try:
            return self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Error getting transform from {parent_frame} to {child_frame}: {e}")
            return None

    def calculate_board_transform(self, marker_1, marker_2, marker_3):
        # Convert translation vectors to numpy arrays
        p1 = np.array([marker_1.transform.translation.x,
                       marker_1.transform.translation.y,
                       marker_1.transform.translation.z])
        p2 = np.array([marker_2.transform.translation.x,
                       marker_2.transform.translation.y,
                       marker_2.transform.translation.z])
        p3 = np.array([marker_3.transform.translation.x,
                       marker_3.transform.translation.y,
                       marker_3.transform.translation.z])

        # Calculate x-axis direction (from marker 1 to marker 2)
        x_dir = p2 - p1
        x_axis = x_dir / np.linalg.norm(x_dir)

        # Calculate y-axis direction (from marker 2 to marker 3)
        y_dir = p3 - p2
        y_axis = y_dir / np.linalg.norm(y_dir)

        # Calculate z-axis as the cross product of x and y (perpendicular to the board)
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis)

        # Ensure y-axis is orthogonal using the corrected cross product
        y_axis = np.cross(z_axis, x_axis)

        # Construct the rotation matrix
        rotation_matrix = np.vstack([x_axis, y_axis, z_axis]).T

        # Convert the rotation matrix to a quaternion
        quaternion = R.from_matrix(rotation_matrix).as_quat()

        # Translation for the board frame (same as marker_1 position)
        translation = p1

        return translation, quaternion

    def publish_board_frame(self):
        # Get the transforms for the markers with IDs 1, 2, and 3
        marker_1 = self.get_transform("camera_color_optical_frame", "marker_1")
        marker_2 = self.get_transform("camera_color_optical_frame", "marker_2")
        marker_3 = self.get_transform("camera_color_optical_frame", "marker_3")

        if marker_1 and marker_2 and marker_3:
            translation, quaternion = self.calculate_board_transform(marker_1, marker_2, marker_3)

            # Create a TransformStamped message
            board_transform = TransformStamped()
            board_transform.header.stamp = self.get_clock().now().to_msg()
            board_transform.header.frame_id = "camera_color_optical_frame"
            board_transform.child_frame_id = "board_frame"

            # Set the translation and rotation
            board_transform.transform.translation.x = translation[0]
            board_transform.transform.translation.y = translation[1]
            board_transform.transform.translation.z = translation[2]
            board_transform.transform.rotation.x = quaternion[0]
            board_transform.transform.rotation.y = quaternion[1]
            board_transform.transform.rotation.z = quaternion[2]
            board_transform.transform.rotation.w = quaternion[3]

            # Publish the transform
            self.tf_broadcaster.sendTransform(board_transform)
            self.get_logger().info("Board frame published successfully")

def main(args=None):
    rclpy.init(args=args)
    board_publisher = BoardPublisher()

    try:
        rclpy.spin(board_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        board_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
