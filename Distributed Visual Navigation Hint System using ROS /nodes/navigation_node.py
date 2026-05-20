#!/usr/bin/env python3
"""
=====================================================================
  NAVIGATION DECISION NODE  —  ROS2 (Jazzy)
  Task: Distributed Visual Navigation Hint System
  Student Node: Node 4.7 — Navigation Decision
=====================================================================

ROS1 vs ROS2 key differences (referenced throughout this file):
  - ROS1 uses rospy,  ROS2 uses rclpy
  - ROS1: rospy.init_node('name')        ROS2: rclpy.init() + Node class
  - ROS1: rospy.Subscriber(...)          ROS2: self.create_subscription(...)
  - ROS1: rospy.Publisher(...)           ROS2: self.create_publisher(...)
  - ROS1: rospy.get_param(...)           ROS2: self.declare_parameter(...) then get_parameter(...)
  - ROS1: rospy.spin()                   ROS2: rclpy.spin(node)
  - ROS1: rospy.logwarn(...)             ROS2: self.get_logger().warn(...)
=====================================================================
"""

# ROS1 DIFFERENCE: In ROS1, the core Python library is called 'rospy'. You would use `import rospy` instead.
import rclpy
from rclpy.node import Node

# ROS1 equivalent: from std_msgs.msg import String
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import json

class NavigationDecisionNode(Node):
    def __init__(self):
        # ROS1 DIFFERENCE: In ROS1, you initialize the node using `rospy.init_node()`.
        # ROS2 initializes it via the parent class constructor here.
        super().__init__('navigation_node')
        
        # ── Parameters ────────────────────────────────────────────────────────
        # ROS1 DIFFERENCE: ROS1 uses a global "Parameter Server" (rospy.get_param). 
        # ROS2 ties parameters directly to the specific node, requiring declaration first.
        #  Parameters: safety_distance
        self.declare_parameter('safety_distance', 1.5) # 1.5 meters as safety threshold
        self.safety_distance = self.get_parameter('safety_distance').value
        
        self.bridge = CvBridge()
        
        # ── State Variables ───────────────────────────────────────────────────
        # We store the latest data from the 3 topics here
        self.latest_motion = {"direction": "unknown", "unreliable": False}
        self.latest_objects = "None"
        self.latest_depth_center = 999.0  # Default to very far
        
        # Flag to prevent false alarms before camera data arrives
        self.data_received = False
        
        # ── Subscribers ───────────────────────────────────────────────────────
        #  Subscribed: /camera_motion, /object_data, /depth_data
        # ROS1 DIFFERENCE: ROS2 requires a QoS profile (the '10').
        self.sub_motion = self.create_subscription(String, '/camera_motion', self.motion_callback, 10)
        self.sub_obj = self.create_subscription(String, '/object_data', self.obj_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/depth_data', self.depth_callback, 10)
        
        # ── Publisher ─────────────────────────────────────────────────────────
        #  Topics Published: /navigation_command
        self.pub_nav = self.create_publisher(String, '/navigation_command', 10)
        
        # ── Timer ─────────────────────────────────────────────────────────────
        # Process decision at 5 FPS  Minimum: 5 FPS
        # ROS1 DIFFERENCE: ROS1 timers use `rospy.Timer`.
        self.timer = self.create_timer(0.2, self.analyze_navigation)
        
        self.get_logger().info(f"Navigation Decision Node Started. Safety Distance: {self.safety_distance}m")

    # ── Callbacks to store latest data ────────────────────────────────────────
    def motion_callback(self, msg: String):
        self.data_received = True
        try:
            self.latest_motion = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def obj_callback(self, msg: String):
        self.data_received = True
        self.latest_objects = msg.data

    def depth_callback(self, msg: Image):
        self.data_received = True
        try:
            # Convert ROS Image to OpenCV matrix
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # Extract depth from the center of the image
            height, width = cv_image.shape[:2]
            self.latest_depth_center = float(cv_image[height//2, width//2])
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    # ── Main Logic ────────────────────────────────────────────────────────────
    def analyze_navigation(self):
        if not self.data_received:
            return  # Wait for data
            
        command = "move forward"
        reason = "Path is clear"
        
        is_unreliable = self.latest_motion.get('unreliable', False)
        obstacle_detected = (self.latest_objects != "None" and self.latest_objects != "")
        too_close = (0 < self.latest_depth_center < self.safety_distance)

        #  Internal Logic: Decide move left, move right, stop
        #  Failure Handling: If motion unreliable -> STOP
        if is_unreliable:
            command = "stop"
            reason = "Unreliable camera motion (blur/dynamic scene)"
            
        elif too_close or obstacle_detected:
            # If there is an obstacle or we are too close to a wall
            # Decide to avoid it. If we were moving right, turn left. Otherwise turn right.
            if self.latest_motion.get('direction') == 'right':
                command = "move left"
            else:
                command = "move right"
            reason = f"Obstacle/Wall ahead! Distance: {self.latest_depth_center:.2f}m"

        # ── Publish Decision ──────────────────────────────────────────────────
        decision_msg = {
            "command": command,
            "reason": reason,
            "depth_m": round(self.latest_depth_center, 2)
        }
        
        msg = String()
        msg.data = json.dumps(decision_msg)
        self.pub_nav.publish(msg)
        
        # Logging for terminal
        # ROS1 DIFFERENCE: Logging binds to the specific node `self.get_logger()`.
        if command == "stop":
            self.get_logger().error(f"[DECISION: {command.upper()}] {reason}")
        elif command != "move forward":
            self.get_logger().warn(f"[DECISION: {command.upper()}] {reason}")
        else:
            self.get_logger().info(f"[DECISION: {command.upper()}] {reason}")

def main(args=None):
    # ROS1 DIFFERENCE: Explicit initialization required.
    rclpy.init(args=args)
    node = NavigationDecisionNode()
    
    try:
        # ROS1 DIFFERENCE: Explicitly spin the node instance.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()