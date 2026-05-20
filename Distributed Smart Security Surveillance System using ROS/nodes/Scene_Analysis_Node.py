import rclpy
from rclpy.node import Node
from custom_msgs.msg import Object, Objects
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class SceneAnalyzer(Node):
    def __init__(self):
        super().__init__('scene_analyzer')

        # ── Parameter ──────────────────────────────────────────────────
        self.declare_parameter('danger_distance', 1.5)
        self.danger_distance = self.get_parameter('danger_distance').value

        # ── Subscribers ────────────────────────────────────────────────
        self.detected_sub = self.create_subscription(
            Objects,
            '/detected_objects',
            self.detected_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/object_depth',
            self.depth_callback,
            10
        )

        # ── Publisher ──────────────────────────────────────────────────
        self.analysis_pub = self.create_publisher(Objects, '/scene_analysis', 10)

        # ── Internal state ─────────────────────────────────────────────
        self.latest_depth_image = None   # stores the latest depth frame
        self.bridge = CvBridge()         # converts ROS Image → numpy array

        self.get_logger().info(
            f'Scene Analyzer started. Danger distance: {self.danger_distance}m'
        )

    # ──────────────────────────────────────────────────────────────────
    def depth_callback(self, msg):
        """Store the latest depth image whenever Node 3 publishes one."""
        try:
            # Convert ROS Image message → numpy array (float32, values = metres)
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')

    # ──────────────────────────────────────────────────────────────────
    def detected_callback(self, msg):
        """
        Called every time YOLO publishes new detections.
        Filters objects to only those closer than danger_distance.
        """
        if self.latest_depth_image is None:
            self.get_logger().warn('No depth image received yet — skipping analysis.')
            return

        close_objects = Objects()   # this will be our filtered output

        for obj in msg.objects:
            distance = self.get_object_distance(obj)

            if distance is None:
                self.get_logger().warn(
                    f'Could not read depth for {obj.cls_name} — skipping.'
                )
                continue

            self.get_logger().info(
                f'{obj.cls_name} detected at {distance:.2f}m '
                f'(threshold: {self.danger_distance}m)'
            )

            # ── Core filter: only keep objects closer than danger_distance ──
            if distance < self.danger_distance:
                close_objects.objects.append(obj)
                self.get_logger().warn(
                    f'DANGER: {obj.cls_name} is {distance:.2f}m away!'
                )

        # Publish the filtered list (may be empty if nothing is close)
        self.analysis_pub.publish(close_objects)
        self.get_logger().info(
            f'Published {len(close_objects.objects)} close object(s) '
            f'out of {len(msg.objects)} detected.'
        )

    # ──────────────────────────────────────────────────────────────────
    def get_object_distance(self, obj):
        """
        Sample the depth image at the CENTER of the object's bounding box.
        Returns distance in metres, or None if out of bounds / invalid.

        Bounding box fields:
          obj.x, obj.y  = top-left corner (pixels)
          obj.w, obj.h  = width and height (pixels)
        """
        # Calculate center pixel of the bounding box
        center_x = obj.x + obj.w // 2
        center_y = obj.y + obj.h // 2

        img_h, img_w = self.latest_depth_image.shape[:2]

        # Safety check: make sure center pixel is inside the image
        if not (0 <= center_x < img_w and 0 <= center_y < img_h):
            self.get_logger().warn(
                f'Bounding box center ({center_x},{center_y}) is outside '
                f'image bounds ({img_w}x{img_h})'
            )
            return None

        # Read the depth value at the center pixel
        distance = float(self.latest_depth_image[center_y, center_x])

        # Ignore invalid depth readings (0 or NaN means sensor had no data)
        if distance <= 0 or np.isnan(distance):
            return None

        return distance


# ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = SceneAnalyzer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()