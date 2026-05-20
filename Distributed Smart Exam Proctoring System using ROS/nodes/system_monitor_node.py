#                                           Name: Marina Said Mikhail
#                                               ID: 21011026
"""
=====================================================================
  SYSTEM MONITOR NODE  —  ROS2 Jazzy
  Task: Distributed Smart Exam Proctoring System
  Student Node: 4.8 — System Monitor
=====================================================================

Purpose:
- Subscribe to all available topics in the current system
- Track whether each node is publishing correctly
- Display a full live summary of the pipeline
- Help debugging if a topic stops publishing

Current team topics detected from teammates' code:
- /camera_frames        -> sensor_msgs/Image
- /face_data            -> sensor_msgs/Image
- /face_coordinates     -> std_msgs/String
- /object_data          -> std_msgs/String
- /object_depth         -> sensor_msgs/Image
- /behavior_state       -> std_msgs/String

Future topics from full task:
- /violation_event      -> not implemented yet
- /alert_status         -> not implemented yet
=====================================================================
"""

import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor_node')

        # ---------------- Parameters ----------------
        self.declare_parameter('timeout_sec', 3.0)          # if no message received within this time => OFFLINE
        self.declare_parameter('monitor_period_sec', 1.0)   # print status every 1 second

        self.timeout_sec = self.get_parameter('timeout_sec').value
        self.monitor_period = self.get_parameter('monitor_period_sec').value

        # ---------------- Helpers ----------------
        self.bridge = CvBridge()

        # ---------------- State tracking ----------------
        # Last received time for each topic
        self.last_seen = {
            '/camera_frames': None,
            '/face_data': None,
            '/face_coordinates': None,
            '/object_data': None,
            '/object_depth': None,
            '/behavior_state': None,
            '/violation_event': None,   # future node
            '/alert_status': None       # future node
        }

        # Message counters
        self.msg_count = {
            '/camera_frames': 0,
            '/face_data': 0,
            '/face_coordinates': 0,
            '/object_data': 0,
            '/object_depth': 0,
            '/behavior_state': 0,
            '/violation_event': 0,
            '/alert_status': 0
        }

        # Latest summaries
        self.latest_frame_id = "None"
        self.latest_face_count = 0
        self.latest_face_types = []
        self.latest_object_text = "None"
        self.latest_object_count = 0
        self.latest_depth_info = "No depth yet"
        self.latest_behavior = "No behavior state yet"

        # ---------------- Subscribers ----------------
        self.sub_camera = self.create_subscription(
            Image,
            '/camera_frames',
            self.camera_callback,
            10
        )

        self.sub_face_preview = self.create_subscription(
            Image,
            '/face_data',
            self.face_preview_callback,
            10
        )

        self.sub_face_coords = self.create_subscription(
            String,
            '/face_coordinates',
            self.face_coordinates_callback,
            10
        )

        self.sub_object = self.create_subscription(
            String,
            '/object_data',
            self.object_callback,
            10
        )

        self.sub_depth = self.create_subscription(
            Image,
            '/object_depth',
            self.depth_callback,
            10
        )

        self.sub_behavior = self.create_subscription(
            String,
            '/behavior_state',
            self.behavior_callback,
            10
        )

        # ---------------- Timer ----------------
        self.timer = self.create_timer(self.monitor_period, self.print_system_status)

        self.get_logger().info("System Monitor Node started successfully.")
        self.get_logger().info("Monitoring all currently implemented topics...")

    # =========================================================
    # Utility functions
    # =========================================================
    def mark_seen(self, topic_name: str):
        self.last_seen[topic_name] = self.get_clock().now()
        self.msg_count[topic_name] += 1

    def get_topic_status(self, topic_name: str) -> str:
        """
        Returns:
        - WAITING  -> never received anything
        - ONLINE   -> received recently
        - OFFLINE  -> received before, but timed out
        """
        if self.last_seen[topic_name] is None:
            return "WAITING"

        elapsed = (self.get_clock().now() - self.last_seen[topic_name]).nanoseconds / 1e9
        if elapsed <= self.timeout_sec:
            return "ONLINE"
        return "OFFLINE"

    def safe_json_loads(self, text: str, default):
        try:
            return json.loads(text)
        except Exception:
            return default

    # =========================================================
    # Callbacks
    # =========================================================
    def camera_callback(self, msg: Image):
        self.mark_seen('/camera_frames')
        self.latest_frame_id = msg.header.frame_id if msg.header.frame_id else "No frame_id"

    def face_preview_callback(self, msg: Image):
        self.mark_seen('/face_data')
        # no need to decode image for monitor, only confirm reception

    def face_coordinates_callback(self, msg: String):
        self.mark_seen('/face_coordinates')

        faces = self.safe_json_loads(msg.data, [])
        if isinstance(faces, list):
            self.latest_face_count = len(faces)
            self.latest_face_types = [face.get('type', 'unknown') for face in faces if isinstance(face, dict)]
        else:
            self.latest_face_count = 0
            self.latest_face_types = []

    def object_callback(self, msg: String):
        self.mark_seen('/object_data')
        self.latest_object_text = msg.data.strip()

        if self.latest_object_text == "None" or self.latest_object_text == "":
            self.latest_object_count = 0
        else:
            self.latest_object_count = len(self.latest_object_text.split('|'))

    def depth_callback(self, msg: Image):
        self.mark_seen('/object_depth')

        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Basic summary of depth image
            h, w = depth_img.shape[:2]

            center_y = h // 2
            center_x = w // 2
            center_value = float(depth_img[center_y, center_x])

            if math.isnan(center_value):
                self.latest_depth_info = f"Depth received ({w}x{h}), center=NaN"
            else:
                self.latest_depth_info = f"Depth received ({w}x{h}), center={center_value:.2f} m"
        except Exception as e:
            self.latest_depth_info = f"Depth received but conversion failed: {str(e)}"

    def behavior_callback(self, msg: String):
        self.mark_seen('/behavior_state')

        behavior = self.safe_json_loads(msg.data, {})
        if isinstance(behavior, dict):
            looking_away = behavior.get("looking_away", False)
            object_usage = behavior.get("object_usage", False)
            unusual_distance = behavior.get("unusual_distance", False)
            distance = behavior.get("distance", -1)

            self.latest_behavior = (
                f"looking_away={looking_away}, "
                f"object_usage={object_usage}, "
                f"unusual_distance={unusual_distance}, "
                f"distance={distance}"
            )
        else:
            self.latest_behavior = msg.data

    # =========================================================
    # Main monitor display
    # =========================================================
    def print_system_status(self):
        camera_status = self.get_topic_status('/camera_frames')
        face_data_status = self.get_topic_status('/face_data')
        face_coords_status = self.get_topic_status('/face_coordinates')
        object_status = self.get_topic_status('/object_data')
        depth_status = self.get_topic_status('/object_depth')
        behavior_status = self.get_topic_status('/behavior_state')
        violation_status = self.get_topic_status('/violation_event')
        alert_status = self.get_topic_status('/alert_status')

        print("\n" + "=" * 78)
        print("                    DISTRIBUTED SMART EXAM SYSTEM MONITOR")
        print("=" * 78)

        print("\n[TOPIC STATUS]")
        print(f"/camera_frames      : {camera_status:<8} | messages: {self.msg_count['/camera_frames']}")
        print(f"/face_data          : {face_data_status:<8} | messages: {self.msg_count['/face_data']}")
        print(f"/face_coordinates   : {face_coords_status:<8} | messages: {self.msg_count['/face_coordinates']}")
        print(f"/object_data        : {object_status:<8} | messages: {self.msg_count['/object_data']}")
        print(f"/object_depth       : {depth_status:<8} | messages: {self.msg_count['/object_depth']}")
        print(f"/behavior_state     : {behavior_status:<8} | messages: {self.msg_count['/behavior_state']}")
        print(f"/violation_event    : {violation_status:<8} | messages: {self.msg_count['/violation_event']}  (not implemented yet)")
        print(f"/alert_status       : {alert_status:<8} | messages: {self.msg_count['/alert_status']}   (not implemented yet)")

        print("\n[LATEST SUMMARY]")
        print(f"Latest frame id     : {self.latest_frame_id}")
        print(f"Faces detected      : {self.latest_face_count}")
        print(f"Face types          : {self.latest_face_types if self.latest_face_types else 'None'}")
        print(f"Objects detected    : {self.latest_object_count}")
        print(f"Object data         : {self.latest_object_text}")
        print(f"Depth summary       : {self.latest_depth_info}")
        print(f"Behavior state      : {self.latest_behavior}")

        print("\n[SYSTEM HEALTH CHECK]")
        active_main_topics = [
            camera_status,
            face_coords_status,
            object_status,
            depth_status,
            behavior_status
        ]

        if all(status == "ONLINE" for status in active_main_topics):
            print("System status       : HEALTHY")
        elif any(status == "OFFLINE" for status in active_main_topics):
            print("System status       : WARNING - one or more active nodes stopped publishing")
        else:
            print("System status       : STARTING / WAITING FOR DATA")

        print("=" * 78)

        # Optional ROS logs
        if camera_status == "OFFLINE":
            self.get_logger().warn("Camera stream seems offline.")
        if face_coords_status == "OFFLINE":
            self.get_logger().warn("Face coordinates topic seems offline.")
        if object_status == "OFFLINE":
            self.get_logger().warn("Object detection topic seems offline.")
        if depth_status == "OFFLINE":
            self.get_logger().warn("Depth estimation topic seems offline.")
        if behavior_status == "OFFLINE":
            self.get_logger().warn("Behavior analysis topic seems offline.")


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()