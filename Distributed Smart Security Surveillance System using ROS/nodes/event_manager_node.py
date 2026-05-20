#!/usr/bin/env python3
"""
=====================================================================
  EVENT MANAGER NODE  —  ROS2 (Jazzy)
  Task: Distributed Smart Security Surveillance System
  Student Node: Node 4.5 — Event Manager
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

# Standard message type — we use String and encode our data as JSON inside it.
# ROS1 equivalent:  from std_msgs.msg import String
from std_msgs.msg import String

import json   # to pack/unpack data as JSON strings
import time   # for Unix timestamps on events


class EventManagerNode(Node):
    """
    Event Manager Node
    ------------------
    Subscribes to  :  /scene_analysis    (from Scene Analysis node)
    Publishes to   :  /security_event    (to Security Response & Logger nodes)
    Parameter      :  restricted_objects (list of object labels to watch)

    Detects two types of security events:
      1. A restricted object (e.g. 'person', 'knife') appears in the frame
      2. Any object is flagged as too close to the camera
    """

    def __init__(self):
        # ── Initialise the ROS2 node ──────────────────────────────
        # This registers the node with ROS2 under the name 'event_manager'.
        # ROS1 equivalent:  rospy.init_node('event_manager', anonymous=False)
        # ROS1 DIFFERENCE: In ROS1, you initialize the node using `rospy.init_node()`, usually down in the main block. 
        # ROS2 initializes it via the parent class constructor here.
        super().__init__('event_manager')

        # ── Declare & read parameters ─────────────────────────────
        # In ROS2 you MUST declare a parameter before reading it.
        # ROS1 equivalent:
        #   restricted = rospy.get_param('~restricted_objects', ['person', 'knife'])
        # ROS1 DIFFERENCE: ROS1 uses a global "Parameter Server". You could just call `rospy.get_param()` anywhere.
        # ROS2 ties parameters directly to the specific node, requiring you to declare them first.
        self.declare_parameter(
            'restricted_objects',
            ['person', 'backpack', 'suitcase']   # default — can be overridden at launch
        )
        self.restricted_objects = (
            self.get_parameter('restricted_objects')
                .get_parameter_value()
                .string_array_value
        )
        
        # ROS1 DIFFERENCE: Logging in ROS1 uses global functions like `rospy.loginfo()`. 
        # ROS2 binds the logger directly to your node object (`self.get_logger()`).
        self.get_logger().info(
            f'[EventManager] Watching for restricted objects: {self.restricted_objects}'
        )

        # ── Create subscriber to /scene_analysis ──────────────────
        # Every time a message arrives on this topic our callback runs.
        # ROS1 equivalent:
        #   rospy.Subscriber('/scene_analysis', String, self.scene_callback)
        # ROS1 DIFFERENCE: ROS2 requires you to specify a "Quality of Service" (QoS) profile. The '10' here is the history queue size.
        self.scene_sub = self.create_subscription(
            String,              # message type
            '/scene_analysis',   # topic to listen on
            self.scene_callback, # function called for each message
            10                   # queue depth  (ROS1: queue_size=10)
        )

        # ── Create publisher to /security_event ───────────────────
        # ROS1 equivalent:
        #   self.pub = rospy.Publisher('/security_event', String, queue_size=10)
        # ROS1 DIFFERENCE: The syntax in ROS1 would be `rospy.Publisher('/security_event', String, queue_size=10)`.
        self.event_pub = self.create_publisher(
            String,
            '/security_event',
            10
        )

        self.get_logger().info('[EventManager] Ready. Waiting for /scene_analysis ...')

    # ─────────────────────────────────────────────────────────────
    def scene_callback(self, msg: String):
        """
        Called automatically each time a /scene_analysis message arrives.

        Expected JSON format from Scene Analysis node:
        {
          "objects": [
            {"label": "person", "confidence": 0.91, "depth": 1.2, "too_close": true},
            {"label": "chair",  "confidence": 0.75, "depth": 3.5, "too_close": false}
          ],
          "frame_id": 42
        }
        """
        # Parse incoming JSON
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(
                '[EventManager] Received non-JSON message on /scene_analysis, skipping.'
            )
            return

        objects  = data.get('objects', [])
        frame_id = data.get('frame_id', -1)

        # Check every detected object for security events
        for obj in objects:
            label      = obj.get('label', 'unknown')
            confidence = obj.get('confidence', 0.0)
            depth      = obj.get('depth', 999.0)
            too_close  = obj.get('too_close', False)

            # ── Event 1: Restricted object in scene ───────────────
            if label in self.restricted_objects:
                self._publish_event(
                    event_type  = 'restricted_object',
                    label       = label,
                    confidence  = confidence,
                    depth       = depth,
                    frame_id    = frame_id,
                    description = f'Restricted object "{label}" detected in scene'
                )

            # ── Event 2: Object dangerously close to camera ───────
            if too_close:
                self._publish_event(
                    event_type  = 'object_too_close',
                    label       = label,
                    confidence  = confidence,
                    depth       = depth,
                    frame_id    = frame_id,
                    description = f'"{label}" is too close to camera (depth={depth:.2f}m)'
                )

    # ─────────────────────────────────────────────────────────────
    def _publish_event(self, event_type, label, confidence,
                       depth, frame_id, description):
        """Build, log, and publish one security event to /security_event."""

        event = {
            'event_type':  event_type,
            'label':       label,
            'confidence':  round(confidence, 3),
            'depth':       round(depth, 3),
            'frame_id':    frame_id,
            'description': description,
            'timestamp':   time.time()      # Unix time — logger node can format this
        }

        # Wrap JSON in a std_msgs/String and publish
        ros_msg = String()
        ros_msg.data = json.dumps(event)
        self.event_pub.publish(ros_msg)

        # Print to terminal so you can see events live while testing
        # ROS1 equivalent:  rospy.logwarn(...)
        # ROS1 DIFFERENCE: Uses the node-specific logger instead of the global `rospy.logwarn()`.
        self.get_logger().warn(
            f'[SECURITY EVENT] type={event_type} | object={label} | '
            f'conf={confidence:.2f} | depth={depth:.2f}m | frame={frame_id}'
        )


# ─────────────────────────────────────────────────────────────────
def main(args=None):
    """
    Entry point — invoked by:
        ros2 run your_package event_manager
    """
    # Initialise ROS2 Python client library
    # ROS1: handled automatically inside rospy.init_node()
    # ROS1 DIFFERENCE: ROS1 does not require an explicit library initialization function like `rclpy.init()`.
    rclpy.init(args=args)

    node = EventManagerNode()

    # Block here, processing callbacks until Ctrl+C
    # ROS1 equivalent:  rospy.spin()
    try:
        # ROS1 DIFFERENCE: ROS1 uses `rospy.spin()`, which pauses the whole script and listens to all topics. 
        # ROS2 explicitly spins your specific node instance `rclpy.spin(node)`.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[EventManager] Shutdown requested (Ctrl+C).')
    finally:
        # ROS1 DIFFERENCE: ROS1 automatically cleans up when the script stops. ROS2 expects explicit cleanup.
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()