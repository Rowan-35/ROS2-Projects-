#!/usr/bin/env python3
"""
=====================================================================
  EVENT LOGGER NODE  —  ROS2 (Jazzy)
  Task: Distributed Smart Security Surveillance System
  Student Node: Node 4.7 — Event Logger
=====================================================================

ROS1 vs ROS2 key differences (referenced throughout this file):
  - ROS1 uses rospy,  ROS2 uses rclpy
  - ROS1: rospy.init_node('name')        ROS2: rclpy.init() + Node class
  - ROS1: rospy.Subscriber(...)          ROS2: self.create_subscription(...)
  - ROS1: rospy.Publisher(...)           ROS2: self.create_publisher(...)
  - ROS1: rospy.get_param(...)           ROS2: self.declare_parameter(...) then get_parameter(...)
  - ROS1: rospy.spin()                   ROS2: rclpy.spin(node)
  - ROS1: rospy.loginfo(...)             ROS2: self.get_logger().info(...)
  - ROS1 build system: catkin            ROS2 build system: colcon
=====================================================================
"""

# ── ROS2 imports ──────────────────────────────────────────────────
# ROS1 equivalent would be:  import rospy
import rclpy

# ROS1 DIFFERENCE: ROS2 is heavily object-oriented. You inherit from a base `Node` class. 
# In ROS1, this is optional; you can just write procedural code without wrapping it in a class.
from rclpy.node import Node

# Standard message type — we use String to pass JSON data between nodes.
# ROS1 equivalent:  from std_msgs.msg import String
from std_msgs.msg import String

import json
import os
from datetime import datetime


class EventLoggerNode(Node):
    """
    Event Logger Node
    -----------------
    Subscribes to  :  /security_event (from Event Manager Node)
    Subscribes to  :  /security_alert (from Security Response Node)
    Parameter      :  log_file_name   (name of the file where logs are saved)

    Responsibility:
      Receives JSON strings from the system, prints them clearly to the terminal,
      and permanently saves them to a text file for historical record-keeping.
    """

    def __init__(self):
        # ── Initialise the ROS2 node ──────────────────────────────
        # ROS1 DIFFERENCE: In ROS1, you initialize the node using `rospy.init_node()`, usually down in the main block. 
        # ROS2 initializes it via the parent class constructor here.
        super().__init__('event_logger')

        # ── Declare & read parameters ─────────────────────────────
        # ROS1 DIFFERENCE: ROS1 uses a global "Parameter Server". You could just call `rospy.get_param()` anywhere.
        # ROS2 ties parameters directly to the specific node, requiring you to declare them first.
        self.declare_parameter('log_file_name', 'surveillance_system_log.txt')
        self.log_file_name = self.get_parameter('log_file_name').get_parameter_value().string_value

        # Set up the file path (saves in the current working directory of the terminal)
        self.log_file_path = os.path.join(os.getcwd(), self.log_file_name)
        
        # ROS1 DIFFERENCE: Logging in ROS1 uses global functions like `rospy.loginfo()`. 
        # ROS2 binds the logger directly to your node object (`self.get_logger()`).
        self.get_logger().info(f'[EventLogger] Started. Writing logs to: {self.log_file_path}')

        # ── Create Subscribers ────────────────────────────────────
        # ROS1 DIFFERENCE: ROS2 requires you to specify a "Quality of Service" (QoS) profile. The '10' here is the history queue size.
        
        # Subscriber 1: Listens to the Event Manager
        self.event_sub = self.create_subscription(
            String,
            '/security_event',
            self.security_event_callback,
            10
        )

        # Subscriber 2: Listens to the Security Response node
        self.alert_sub = self.create_subscription(
            String,
            '/security_alert',
            self.security_alert_callback,
            10
        )

    # ─────────────────────────────────────────────────────────────
    def security_event_callback(self, msg: String):
        """Processes messages from the Event Manager Node"""
        try:
            data = json.loads(msg.data)
            description = data.get('description', 'Unknown event detected')
            event_type = data.get('event_type', 'UNKNOWN_TYPE')
            
            log_string = f"[EVENT] [{event_type}] {description}"
            
            # Print to terminal
            self.get_logger().info(log_string)
            
            # Save to file
            self._write_to_file(log_string)
            
        except json.JSONDecodeError:
            self.get_logger().error('[EventLogger] Failed to decode JSON from /security_event')

    # ─────────────────────────────────────────────────────────────
    def security_alert_callback(self, msg: String):
        """Processes messages from the Security Response Node"""
        try:
            data = json.loads(msg.data)
            action_taken = data.get('action', 'Unknown action triggered')
            alert_level = data.get('alert_level', 'WARNING')
            
            log_string = f"[ALERT] [{alert_level}] ACTION TAKEN: {action_taken}"
            
            # Print to terminal as a WARNING (yellow text in ROS2)
            self.get_logger().warn(log_string)
            
            # Save to file
            self._write_to_file(log_string)
            
        except json.JSONDecodeError:
            self.get_logger().error('[EventLogger] Failed to decode JSON from /security_alert')

    # ─────────────────────────────────────────────────────────────
    def _write_to_file(self, log_message: str):
        """Helper method to append a timestamped message to the text file."""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        formatted_log = f"[{timestamp}] {log_message}\n"
        
        try:
            with open(self.log_file_path, 'a') as file:
                file.write(formatted_log)
        except Exception as e:
            self.get_logger().error(f'[EventLogger] Failed to write to log file: {e}')


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    """
    Entry point — invoked by:
        ros2 run your_package event_logger
    """
    # ROS1 DIFFERENCE: ROS1 does not require an explicit library initialization function like `rclpy.init()`.
    rclpy.init(args=args)

    node = EventLoggerNode()

    try:
        # ROS1 DIFFERENCE: ROS1 uses `rospy.spin()`, which pauses the whole script and listens to all topics. 
        # ROS2 explicitly spins your specific node instance `rclpy.spin(node)`.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[EventLogger] Shutdown requested (Ctrl+C).')
    finally:
        # ROS1 DIFFERENCE: ROS1 automatically cleans up when the script stops. ROS2 expects explicit cleanup.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()