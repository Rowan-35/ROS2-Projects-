#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from example_interfaces.action import Fibonacci

class SecurityResponseNode(Node):
    def __init__(self):
        super().__init__('security_response_node')

        # Node Logic:
        # Startup: Node initializes& declares parameters, sets up publishers, subscribers, and action client.
        # Parameters control alert level, response duration, auto-ack, retries, and retry delay.
        # Callback group to allow concurrent callbacks (subscriber + action feedback)
        self.cb_group = ReentrantCallbackGroup()

        # Parameters (declare then get)
        self.declare_parameter('alert_level', 'medium')           # default alert level
        self.declare_parameter('response_duration', 10)           # seconds (used as action goal payload)
        self.declare_parameter('auto_ack', True)                  # automatically acknowledge events
        self.declare_parameter('max_retries', 2)                  # retry action on failure
        self.declare_parameter('retry_delay', 2.0)                # seconds between retries

        self.alert_level = self.get_parameter('alert_level').get_parameter_value().string_value
        self.response_duration = int(self.get_parameter('response_duration').get_parameter_value().integer_value)
        self.auto_ack = self.get_parameter('auto_ack').get_parameter_value().bool_value
        self.max_retries = int(self.get_parameter('max_retries').get_parameter_value().integer_value)
        self.retry_delay = float(self.get_parameter('retry_delay').get_parameter_value().double_value)

        # ROS Communication:
        # Publisher: /security_alert (std_msgs/String)
        # Subscriber: /security_event (std_msgs/String)
        # Action Client: /security_action (Fibonacci placeholder)
        # QoS Profile: Reliable, depth 10

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.alert_publisher = self.create_publisher(String, '/security_alert', 10)
        self.event_subscriber = self.create_subscription(
            String,
            '/security_event',
            self.event_callback,
            qos,
            callback_group=self.cb_group
        )

        # Action client for long-running security responses
        self.action_client = ActionClient(self, Fibonacci, '/security_action', callback_group=self.cb_group)

        # Internal state
        self._pending_goal_handle = None
        self._current_retries = 0

        self.get_logger().info('Security Response Node started. Waiting for events...')

    def event_callback(self, msg: String):
        """
        === Node Logic: Event Reception ===
        Called when a /security_event message arrives.
        Parses event text, assesses severity, publishes alert, and optionally triggers action.
        """
        event_text = msg.data or ""
        self.get_logger().info(f"Received security event: '{event_text}'")

        # Basic parsing and decision logic
        severity = self._assess_event_severity(event_text)
        alert_msg = String()
        alert_msg.data = f"ALERT_TRIGGERED|level={severity}|event={event_text}"
        self.alert_publisher.publish(alert_msg)
        self.get_logger().info(f"Published alert: {alert_msg.data}")

        # Response Action: auto-ack launches action, else waits for manual trigger
        if self.auto_ack:
            self.get_logger().info("Auto-ack enabled: launching response action.")
            self._current_retries = 0
            self._send_response_action(severity, event_text)
        else:
            self.get_logger().info("Auto-ack disabled: waiting for manual trigger.")

    def _assess_event_severity(self, event_text: str) -> str:
        """
        === Node Logic: Severity Assessment ===
        Maps event content to alert level (low/medium/high).
        """
        text = event_text.lower()
        if 'intrusion' in text or 'weapon' in text or 'attack' in text:
            return 'high'
        if 'loiter' in text or 'suspicious' in text or 'vandal' in text:
            return 'medium'
        return 'low'

    def _send_response_action(self, severity: str, event_text: str):
        """
        === System Integration: Action Flow ===
        Sends long-running action goal to /security_action.
        Uses response_duration as proxy for workload.
        """
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server '/security_action' not available.")
            fail_msg = String()
            fail_msg.data = f"ALERT_FAILED|reason=action_server_unavailable|event={event_text}"
            self.alert_publisher.publish(fail_msg)
            return

        goal_msg = Fibonacci.Goal()
        goal_msg.order = max(1, self.response_duration)

        self.get_logger().info(f"Sending action goal (duration proxy={goal_msg.order}) for severity={severity}")

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(lambda fut: self._goal_response_callback(fut, event_text, severity))


def main(args=None):
    """
    === Step-by-Step Timeline ===
    1. Startup, Event Reception, Alert Publishing, Response Action, Failure Handling, Shutdown
    2. ROS Communication: Publisher, Subscriber, Action Client, QoS
    3. System Integration: Input, Processing, Output, Action, Resilience
    4. ROS1 vs ROS2 Differences:
       - Node API: rospy vs rclpy Node class
       - Actions: actionlib vs native ActionClient
       - QoS: simple pub/sub vs QoS policies
       - Parameters: global server vs per-node declaration
       - Concurrency: spinners vs callback groups/executors
    """
    rclpy.init(args=args)
    node = SecurityResponseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

