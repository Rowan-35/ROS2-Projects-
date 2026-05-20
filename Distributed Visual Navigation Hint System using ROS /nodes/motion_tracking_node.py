import rclpy
from rclpy.node import Node
from navigation_msgs.msg import ROIFeatures, MotionData
import numpy as np

class MotionTrackingNode(Node):
    """
    ROS 2 Node that tracks motion between consecutive frames
    using the fixed grid of ROIs from /roi_features.
    """

    def __init__(self):
        super().__init__('motion_tracking_node')

        # --- Parameters ---
        self.declare_parameter('motion_threshold', 5.0)
        self.motion_threshold = self.get_parameter('motion_threshold').value

        # --- Subscriber to ROI features ---
        self.subscription = self.create_subscription(
            ROIFeatures,
            '/roi_features',
            self.roi_callback,
            10
        )

        # --- Publisher for motion data ---
        self.publisher = self.create_publisher(MotionData, '/motion_data', 10)

        # --- State ---
        self.prev_centroids_x = None
        self.prev_centroids_y = None

        self.get_logger().info(f"Motion Tracking Node started with motion_threshold = {self.motion_threshold}")

    def roi_callback(self, msg: ROIFeatures):
        """
        Process incoming ROIFeatures message.
        Compares current centroids with previous ones (by index) to compute motion.
        """
        # Convert lists to numpy arrays
        current_x = np.array(msg.centroids_x, dtype=np.float32)
        current_y = np.array(msg.centroids_y, dtype=np.float32)
        current_count = len(current_x)

        # Prepare motion message with current timestamp
        motion_msg = MotionData()
        motion_msg.header.stamp = self.get_clock().now().to_msg()
        motion_msg.header.frame_id = "camera_frame"

        # Initialize with default zero values
        motion_msg.avg_dx = 0.0
        motion_msg.avg_dy = 0.0
        motion_msg.displacements_x = []
        motion_msg.displacements_y = []
        motion_msg.num_matches = 0
        motion_msg.confidence = 0.0

        # If we have previous data and the number of ROIs matches, compute motion
        if (self.prev_centroids_x is not None and
            len(self.prev_centroids_x) == current_count and
            current_count > 0):

            # Compute displacements for each ROI (same index)
            dx = current_x - self.prev_centroids_x
            dy = current_y - self.prev_centroids_y

            # Apply motion threshold: ignore small jitter
            magnitude = np.sqrt(dx**2 + dy**2)
            valid = magnitude >= self.motion_threshold

            if np.any(valid):
                dx_valid = dx[valid]
                dy_valid = dy[valid]

                motion_msg.avg_dx = float(np.mean(dx_valid))
                motion_msg.avg_dy = float(np.mean(dy_valid))
                motion_msg.displacements_x = dx_valid.tolist()
                motion_msg.displacements_y = dy_valid.tolist()
                motion_msg.num_matches = len(dx_valid)
                motion_msg.confidence = float(len(dx_valid)) / float(current_count)
        else:
            if current_count > 0 and self.prev_centroids_x is not None:
                self.get_logger().warn(
                    f"ROI count mismatch: prev {len(self.prev_centroids_x)}, curr {current_count}"
                )

        # Publish the result
        self.publisher.publish(motion_msg)

        # Store current data for next iteration
        self.prev_centroids_x = current_x.copy()
        self.prev_centroids_y = current_y.copy()

def main(args=None):
    rclpy.init(args=args)
    node = MotionTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
