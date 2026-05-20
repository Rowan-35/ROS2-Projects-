#!/usr/bin/env python3
"""
=====================================================================
  RULE EVALUATION NODE  —  ROS2 (Jazzy)
  Task: Distributed Smart Exam Proctoring System
  Student Node: Node 4.6 — Rule Evaluation
=====================================================================

ROS1 vs ROS2 key differences (referenced throughout this file):
  - ROS1 uses rospy,  ROS2 uses rclpy
  - ROS1: rospy.init_node('name')        ROS2: rclpy.init() + Node class
  - ROS1: rospy.Subscriber(...)          ROS2: self.create_subscription(...)
  - ROS1: rospy.Publisher(...)           ROS2: self.create_publisher(...)
  - ROS1: rospy.get_param(...)           ROS2: self.declare_parameter(...) then get_parameter(...)
  - ROS1: rospy.spin()                   ROS2: rclpy.spin(node)
  - ROS1: rospy.loginfo(...)             ROS2: self.get_logger().info(...)
=====================================================================
"""

# ROS1 DIFFERENCE: In ROS1, the core Python library is called 'rospy'. You would use `import rospy` instead.
import rclpy
from rclpy.node import Node

# ROS1 equivalent: from std_msgs.msg import String
from std_msgs.msg import String
import json

class RuleEvaluationNode(Node):
    def __init__(self):
        # ROS1 DIFFERENCE: In ROS1, you initialize the node using `rospy.init_node()`.
        # ROS2 initializes it via the parent class constructor here.
        super().__init__('rule_node')
        
        # ── Parameters ────────────────────────────────────────────────────────
        # ROS1 DIFFERENCE: ROS1 uses a global "Parameter Server". 
        # ROS2 ties parameters directly to the specific node, requiring declaration first.
        self.declare_parameter('violation_rules', ['phone_usage', 'looking_away', 'too_close'])
        self.rules = self.get_parameter('violation_rules').value
        
        # ── Subscriber: Listens to Behavior Analysis (Node 4.5) ───────────────
        # ROS1 DIFFERENCE: ROS2 requires a QoS profile (the '10').
        self.behavior_sub = self.create_subscription(
            String,
            '/behavior_state',
            self.behavior_callback,
            10
        )
        
        # ── Publisher: Sends violations to Alert Node (Node 4.7) ──────────────
        self.violation_pub = self.create_publisher(String, '/violation_event', 10)
        
        self.get_logger().info("Rule Evaluation Node Started. Monitoring behavior rules...")

    def behavior_callback(self, msg: String):
        """
        Called every time Node 4.5 sends a behavior update.
        Decides if these behaviors constitute an exam violation.
        """
        try:
            # Parse the incoming behavior JSON
            behavior_data = json.loads(msg.data)
            
            looking_away = behavior_data.get('looking_away', False)
            object_usage = behavior_data.get('object_usage', False)
            unusual_dist = behavior_data.get('unusual_distance', False)
            
            violation_detected = False
            violation_type = ""
            severity = "LOW"

            # ── Rule Evaluation Logic ─────────────────────────────────────────
            # Rule 1: Using a phone or book is a High Severity violation
            if object_usage:
                violation_detected = True
                violation_type = "PROHIBITED_OBJECT_USAGE"
                severity = "HIGH"
            
            # Rule 2: Looking away for too long
            elif looking_away:
                violation_detected = True
                violation_type = "SUSPICIOUS_LOOKING_AWAY"
                severity = "MEDIUM"
                
            # Rule 3: Being too close to the camera (possible cheating)
            elif unusual_dist:
                violation_detected = True
                violation_type = "UNUSUAL_PROXIMITY"
                severity = "LOW"

            # ── Publish Violation if detected ─────────────────────────────────
            if violation_detected:
                violation_payload = {
                    "violation": violation_type,
                    "severity": severity,
                    "details": behavior_data,
                    "timestamp": self.get_clock().now().to_msg().sec
                }
                
                out_msg = String()
                out_msg.data = json.dumps(violation_payload)
                self.violation_pub.publish(out_msg)
                
                # ROS1 DIFFERENCE: Logging binds to the specific node instance.
                self.get_logger().warn(f"!!! VIOLATION DETECTED: {violation_type} (Severity: {severity}) !!!")
            
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse behavior data.")

def main(args=None):
    # ROS1 DIFFERENCE: Explicit library initialization required.
    rclpy.init(args=args)
    node = RuleEvaluationNode()
    
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