import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from multi_view_interfaces.msg import KeypointArray, Keypoint
import cv2
from cv_bridge import CvBridge
import numpy as np

class KeypointDetectionNode(Node):
    def __init__(self):
        super().__init__('keypoint_detection_node')

        # ---------- Parameters ----------
        self.declare_parameter('max_keypoints', 200)
        self.declare_parameter('detector_type', 'orb')
        self.declare_parameter('scale_factor', 1.2)
        self.declare_parameter('n_levels', 8)
        self.declare_parameter('enable_visualization', False)   # toggle visualisation

        self.max_keypoints = self.get_parameter('max_keypoints').value
        self.detector_type = self.get_parameter('detector_type').value.lower()
        self.scale_factor = self.get_parameter('scale_factor').value
        self.n_levels = self.get_parameter('n_levels').value
        self.enable_vis = self.get_parameter('enable_visualization').value

        # Subscriber
        # ROS2: create_subscription(type, topic, callback, QoS)
        # ROS1: rospy.Subscriber('/camera_frames', Image, self.image_callback)
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera_frames',
            self.image_callback,
            10
        )

        # Publisher
        # ROS2: create_publisher(type, topic, QoS)
        # ROS1: rospy.Publisher('/keypoints', KeypointArray, queue_size=10)
        self.kp_publisher = self.create_publisher(KeypointArray, '/keypoints', 10)

        # Detector 
        if self.detector_type == 'orb':
            self.detector = cv2.ORB_create(
                nfeatures=self.max_keypoints,
                scaleFactor=self.scale_factor,
                nlevels=self.n_levels
            )
        else:
            self.get_logger().warn(
                f"Detector type '{self.detector_type}' not supported, falling back to ORB."
            )
            self.detector = cv2.ORB_create(nfeatures=self.max_keypoints)

        # ROS2 logger (ROS1: rospy.loginfo)
        self.get_logger().info(
            f'Keypoint detection node started. detector={self.detector_type}, '
            f'max_keypoints={self.max_keypoints}, visualization={self.enable_vis}'
        )

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detecting keypoints
        keypoints_cv = self.detector.detect(gray, None)

        # Limiting to max_keypoints
        if len(keypoints_cv) > self.max_keypoints:
            keypoints_cv = keypoints_cv[:self.max_keypoints]

        # System rule: at least 20 keypoints required
        if len(keypoints_cv) < 20:
            self.get_logger().warn(
                f'Only {len(keypoints_cv)} keypoints detected (minimum 20 required). '
                'Possible failure: insufficient features.'
            )

        # Visualization (OpenCV Rich Keypoints)
        if self.enable_vis:
            frame_with_kps = cv2.drawKeypoints(
                frame,
                keypoints_cv,
                None,
                color=(0, 255, 0),                         # green
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
            )
            # Optional: show keypoint count on the image
            cv2.putText(frame_with_kps,
                        f"Keypoints: {len(keypoints_cv)}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2)
            cv2.namedWindow("Keypoints", cv2.WINDOW_NORMAL)
            cv2.imshow("Keypoints", frame_with_kps)
            cv2.waitKey(1)
       

        # Publishing Keypoints
        kp_array = KeypointArray()
        # ROS2: self.get_clock().now().to_msg()
        # ROS1: rospy.Time.now()
        kp_array.header.stamp = self.get_clock().now().to_msg()
        kp_array.header.frame_id = msg.header.frame_id

        for kp in keypoints_cv:
            ros_kp = Keypoint()
            ros_kp.x = kp.pt[0]
            ros_kp.y = kp.pt[1]
            ros_kp.size = kp.size
            ros_kp.angle = kp.angle
            ros_kp.response = kp.response
            ros_kp.octave = kp.octave
            ros_kp.class_id = kp.class_id
            kp_array.keypoints_array.append(ros_kp)

        # ROS2: publisher_.publish(msg)
        # ROS1: publisher.publish(msg)
        self.kp_publisher.publish(kp_array)

        self.get_logger().debug(f'Published {len(keypoints_cv)} keypoints')


def main(args=None):
    rclpy.init(args=args)
    node = KeypointDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.enable_vis:
            cv2.destroyWindow("Keypoints")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

