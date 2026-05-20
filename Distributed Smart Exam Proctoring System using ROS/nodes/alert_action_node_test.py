#!/usr/bin/env python3
"""
alert_action_node.py : test code
Listens for violation events and publishes alert status.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class AlertActionNode(Node):
    def __init__(self):
        super().__init__('alert_action_node')

        # Publisher for alert status
        self.alert_pub = self.create_publisher(String, '/alert_status', 10)

        # Subscriber to violation events from rule_evaluation_node
        self.create_subscription(
            String,
            '/violation_event',
            self.violation_callback,
            10
        )

        self.get_logger().info("Alert Action Node started. Waiting for violation events...")

    def violation_callback(self, msg: String):
        """
        Called whenever a violation is published by rule_evaluation_node.
        Publishes a human-readable alert and logs it.
        """
        try:
            violation_data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse violation JSON")
            return

        violation_type = violation_data.get('violation', 'UNKNOWN')
        severity = violation_data.get('severity', 'UNKNOWN')
        timestamp = violation_data.get('timestamp', '')

        # Format alert message
        alert_msg = String()
        alert_msg.data = (f"[ALERT] {violation_type} | Severity: {severity} | "
                          f"Time: {timestamp}")

        self.alert_pub.publish(alert_msg)
        self.get_logger().warn(f"Alert published: {alert_msg.data}")

        # Optional: You could also trigger external actions here (sound, log file, etc.)

def main(args=None):
    rclpy.init(args=args)
    node = AlertActionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()