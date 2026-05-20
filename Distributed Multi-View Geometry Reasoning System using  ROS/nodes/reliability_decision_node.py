#!/usr/bin/env python3
"""
=====================================================================
  RELIABILITY DECISION NODE  —  ROS2 (Jazzy)
  Task: Distributed Multi-View Geometry Reasoning System
  Student Node: Node 4.8 — Reliability Decision
=====================================================================

ROS1 vs ROS2 key differences (referenced throughout this file):
  - ROS1 uses rospy,  ROS2 uses rclpy
  - ROS1: rospy.init_node('name')        ROS2: rclpy.init() + Node class
  - ROS1: rospy.Subscriber(...)          ROS2: self.create_subscription(...)
  - ROS1: rospy.Publisher(...)           ROS2: self.create_publisher(...)
  - ROS1: rospy.get_param(...)           ROS2: self.declare_parameter(...) then get_parameter(...)
  - ROS1: rospy.spin()                   ROS2: rclpy.spin(node)
  - ROS1: Action servers are handled via actionlib. ROS2 uses rclpy.action.
=====================================================================
"""

# ROS1 DIFFERENCE: In ROS1, the core Python library is called 'rospy'. You would use `import rospy` instead.
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
import json

# 🛠️ CORRECTION: Importing the correct Custom Message Type used by Node 4.6
from multi_view_interfaces.msg import FeatureMatchArray

# Replace with your actual action interface when created
# from multi_view_interfaces.action import ReportAction 

class ReliabilityDecisionNode(Node):

    def __init__(self):
        # ROS1 DIFFERENCE: In ROS1, you initialize the node using `rospy.init_node()`.
        # ROS2 initializes it via the parent class constructor here.
        super().__init__('reliability_decision_node')

        # ── Parameters ──────────────────────────────────────────────────────
        # ROS1 DIFFERENCE: ROS1 uses a global "Parameter Server" (rospy.get_param). 
        # ROS2 ties parameters directly to the specific node, requiring declaration first.
        self.declare_parameter('min_inliers', 8)  # Changed default to 8 based on Node 4.6 minimum[cite: 13]
        self.declare_parameter('min_motion_confidence', 0.4) 

        self.min_inliers = self.get_parameter('min_inliers').value
        self.min_motion_confidence = self.get_parameter('min_motion_confidence').value

        # ── Internal state ───────────────────────────────────────────────────
        self.num_inliers = 0
        self.latest_motion  = None       # From /camera_motion
        self.current_decision = 'UNRELIABLE'

        # ── Subscribers ─────────────────────────────────────────────────────
        # ROS1 DIFFERENCE: ROS2 requires a QoS profile (the '10').
        
        # 🛠️ CORRECTION: Changed message type from String to FeatureMatchArray
        self.sub_inliers = self.create_subscription(
            FeatureMatchArray,
            '/geometric_inliers',
            self.inliers_callback,
            10
        )

        self.sub_motion = self.create_subscription(
            String,
            '/camera_motion',
            self.motion_callback,
            10
        )

        # ── Publisher ────────────────────────────────────────────────────────
        self.pub_state = self.create_publisher(String, '/system_state', 10)

        # ── Action Server ────────────────────────────────────────────────────
        # Requirement 4.8 & 6: Action /report_action[cite: 12]
        # Note: Uncomment when 'ReportAction' interface is ready
        # ROS1 DIFFERENCE: ROS1 uses actionlib.SimpleActionServer. ROS2 uses rclpy.action.ActionServer.
        # self._action_server = ActionServer(
        #     self,
        #     ReportAction,
        #     'report_action',
        #     self.execute_action_callback)

        # ── Timer: 5 Hz (0.2s) as per System Rules ──────────────────────────
        # ROS1 DIFFERENCE: ROS1 timers use `rospy.Timer`.
        self.timer = self.create_timer(0.2, self.evaluate_reliability)

        self.get_logger().info('✅ Reliability Decision Node Started (Waiting for Action Server interface)')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def inliers_callback(self, msg: FeatureMatchArray):
        # 🛠️ CORRECTION: Logic updated to read from the Custom Message structure[cite: 13]
        self.num_inliers = len(msg.matches)

    def motion_callback(self, msg: String):
        try:
            self.latest_motion = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Bad JSON on /camera_motion')

    # ── Action Logic ────────────────────────────────────────────────────────

    def execute_action_callback(self, goal_handle):
        """Handler for the /report_action request."""
        self.get_logger().info('Received report action request...')
        
        # In a real scenario, you would provide feedback here
        goal_handle.succeed()
        
        # Return the current reliability state as the result
        # result = ReportAction.Result()
        # result.status = self.current_decision
        # return result

    # ── Core Logic ──────────────────────────────────────────────────────────

    def evaluate_reliability(self):
        """Evaluate and publish the system state."""
        self.current_decision = self._compute_decision()

        out = String()
        out.data = json.dumps({
            'state': self.current_decision,
            'timestamp': self.get_clock().now().to_msg().sec
        })
        self.pub_state.publish(out)
        
        # ROS1 DIFFERENCE: Logging binds to the specific node instance (`self.get_logger()`).
        if self.current_decision == 'RELIABLE':
            self.get_logger().info(f'State: {self.current_decision}')
        else:
            self.get_logger().warn(f'State: {self.current_decision}')

    def _compute_decision(self) -> str:
        # 1. Check if data exists
        if self.latest_motion is None:
            return 'UNRELIABLE'

        motion_conf    = self.latest_motion.get('confidence', 0.0)
        blur_detected  = self.latest_motion.get('blur_detected', False)

        # Rule 1: Check inlier count[cite: 12]
        if self.num_inliers < self.min_inliers:
            # If inliers are very low, we might lack features[cite: 12]
            if self.num_inliers < 5: 
                return 'LOW_FEATURES'
            return 'UNRELIABLE'

        # Rule 2: Failure handling for blur or low motion confidence[cite: 12]
        if blur_detected or motion_conf < self.min_motion_confidence:
            return 'UNRELIABLE'

        return 'RELIABLE'

def main(args=None):
    # ROS1 DIFFERENCE: Explicit library initialization required.
    rclpy.init(args=args)
    node = ReliabilityDecisionNode()
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