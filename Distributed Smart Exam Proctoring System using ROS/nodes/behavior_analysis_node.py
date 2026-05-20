#!/usr/bin/env python3
"""
=====================================================================
  BEHAVIOR ANALYSIS NODE  —  ROS2 (Jazzy)
  Task: Distributed Smart Exam Proctoring System
  Student Node: Node 4.5 — Behavior Analysis
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

class BehaviorAnalysisNode(Node):
    def __init__(self):
        # ROS1 DIFFERENCE: In ROS1, you initialize the node using `rospy.init_node()`.
        # ROS2 initializes it via the parent class constructor here.
        super().__init__('behavior_node')
        
        # ── Parameters ────────────────────────────────────────────────────────
        # ROS1 DIFFERENCE: ROS1 uses a global "Parameter Server" (rospy.get_param). 
        # ROS2 ties parameters directly to the specific node, requiring declaration first.
        self.declare_parameter('attention_threshold', 1.0) # 1.0 meter as unusual distance threshold
        self.depth_thresh = self.get_parameter('attention_threshold').value
        
        self.bridge = CvBridge()
        
        # ── State Variables ───────────────────────────────────────────────────
        # We store the latest data from the 3 topics here
        self.latest_faces = []
        self.latest_objects = "None"
        self.latest_depth = None
        

        # Flag to prevent false alarms before any camera data is actually received
        self.data_received = False
        
        # ── Subscribers ───────────────────────────────────────────────────────
        # NOTE: We use the actual topic names from teammates' code, not the PDF.
        # ROS1 DIFFERENCE: ROS2 requires a QoS profile (the '10').
        self.sub_face = self.create_subscription(String, '/face_coordinates', self.face_callback, 10)
        self.sub_obj = self.create_subscription(String, '/object_data', self.obj_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/object_depth', self.depth_callback, 10)
        
        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub_behavior = self.create_publisher(String, '/behavior_state', 10)
        
        # ── Timer ─────────────────────────────────────────────────────────────
        # Instead of processing only when one specific message arrives, we check the state 5 times a second (5 FPS).
        # ROS1 DIFFERENCE: ROS1 timers use `rospy.Timer`.
        self.timer = self.create_timer(0.2, self.analyze_behavior)
        
        self.get_logger().info("Behavior Analysis Node Started. Waiting for camera data...")

    # ── Callbacks to store latest data ────────────────────────────────────────
    def face_callback(self, msg: String):
        self.data_received = True  # Activate processing
        try:
            self.latest_faces = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_faces = []

    def obj_callback(self, msg: String):
        self.data_received = True  # Activate processing
        self.latest_objects = msg.data

    def depth_callback(self, msg: Image):
        self.data_received = True  # Activate processing
        # Convert ROS Image to OpenCV matrix to read the depth numbers
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # ── Main Logic ────────────────────────────────────────────────────────────
    def analyze_behavior(self):

        if not self.data_received:
            return  # Do nothing until we actually get data from the camera/other nodes
            
        looking_away = False
        object_usage = False
        unusual_distance = False
        distance_val = -1.0
        
        # Rule 1: Object Usage (Phone or Book detected)
        if self.latest_objects != "None" and self.latest_objects != "":
            object_usage = True
            
        # Rule 2 & 3: Looking Away & Unusual Distance
        front_face_found = False
        
        for face in self.latest_faces:
            if face.get('type') == 'side':
                looking_away = True
            elif face.get('type') == 'front':
                front_face_found = True
                
                # If we have a front face, let's check its distance
                if self.latest_depth is not None:
                    # Get center coordinates of the face bounding box
                    x, y, w, h = face['x'], face['y'], face['w'], face['h']
                    center_x = min(x + w//2, self.latest_depth.shape[1] - 1)
                    center_y = min(y + h//2, self.latest_depth.shape[0] - 1)
                    
                    # Read the exact depth pixel value
                    distance_val = float(self.latest_depth[center_y, center_x])
                    
                    # If distance is less than the threshold (e.g., face is 0.5m from screen) -> Unusual!
                    if 0 < distance_val < self.depth_thresh:
                        unusual_distance = True

        # If no faces are detected at all, the student might have left or is fully turned away
        if not front_face_found and not looking_away:
            looking_away = True

        # ── Publish Behavior State ────────────────────────────────────────────
        state = {
            "looking_away": looking_away,
            "object_usage": object_usage,
            "unusual_distance": unusual_distance,
            "distance": round(distance_val, 2)
        }
        
        msg = String()
        msg.data = json.dumps(state)
        self.pub_behavior.publish(msg)
        
        # Logging for debugging
        # ROS1 DIFFERENCE: Logging binds to the specific node `self.get_logger()`.
        if looking_away or object_usage or unusual_distance:
            self.get_logger().warn(f"[SUSPICIOUS BEHAVIOR] {state}")
        else:
            self.get_logger().info("[NORMAL] Student is focused.")

def main(args=None):
    # ROS1 DIFFERENCE: Explicit initialization required.
    rclpy.init(args=args)
    node = BehaviorAnalysisNode()
    
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
