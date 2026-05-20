# Name :                                Marina Said Mikhail Botros 
#ID :                                             21011026

import rclpy            #instead of(io): import rospy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import time


class SystemMonitorNode(Node):      #io: rospy.init_node('system_monitor')
    def __init__(self):
        super().__init__('system_monitor')  #io: rospy.init_node('system_monitor')


        # Store latest message data:-

        self.last_camera_msg = None
        self.last_detected_objects_msg = None
        self.last_object_depth_msg = None
        self.last_scene_analysis_msg = None
        self.last_security_event_msg = None
        self.last_security_alert_msg = None

        # Store last receive times:-

        self.last_camera_time = None
        self.last_detected_objects_time = None
        self.last_object_depth_time = None
        self.last_scene_analysis_time = None
        self.last_security_event_time = None
        self.last_security_alert_time = None

#---------------------------------------------------------------------------------------------------------------------------------------
        
        # Subscriptions:-
        
        self.camera_sub = self.create_subscription(
            String,
            '/camera_frames',
            self.camera_callback,
            10
        )

            #io: self.camera_sub = rospy.Subscriber(
            #   '/camera_frames',
            #   String,
            #   self.camera_callback,
            #   queue_size=10
            #   )

        self.detected_objects_sub = self.create_subscription(
            String,
            '/detected_objects',
            self.detected_objects_callback,
            10
        )

        self.object_depth_sub = self.create_subscription(
            Float32,
            '/object_depth',
            self.object_depth_callback,
            10
        )

        self.scene_analysis_sub = self.create_subscription(
            String,
            '/scene_analysis',
            self.scene_analysis_callback,
            10
        )

        self.security_event_sub = self.create_subscription(
            String,
            '/security_event',
            self.security_event_callback,
            10
        )

        self.security_alert_sub = self.create_subscription(
            String,
            '/security_alert',
            self.security_alert_callback,
            10
        )

#--------------------------------------------------------------------------------------------------------------------------------------------------
        # Timer for periodic report
        
        self.timer = self.create_timer(2.0, self.print_system_status)
            #io: self.timer = rospy.Timer(rospy.Duration(2.0), self.print_system_status)

        self.get_logger().info('System Monitor Node has started.')

#--------------------------------------------------------------------------------------------------------------------------------------------------
    # Callbacks: JSON ---> PYTHON

    def camera_callback(self, msg):
        self.last_camera_msg = msg.data
        self.last_camera_time = time.time()
        self.get_logger().info(f'Received /camera_frames: {msg.data}')

    def detected_objects_callback(self, msg):
        self.last_detected_objects_msg = msg.data
        self.last_detected_objects_time = time.time()
        self.get_logger().info(f'Received /detected_objects: {msg.data}')

    def object_depth_callback(self, msg):
        self.last_object_depth_msg = msg.data
        self.last_object_depth_time = time.time()
        self.get_logger().info(f'Received /object_depth: {msg.data:.2f}')

    def scene_analysis_callback(self, msg):
        self.last_scene_analysis_msg = msg.data
        self.last_scene_analysis_time = time.time()
        self.get_logger().info(f'Received /scene_analysis: {msg.data}')

    def security_event_callback(self, msg):
        self.last_security_event_msg = msg.data
        self.last_security_event_time = time.time()
        self.get_logger().info(f'Received /security_event: {msg.data}')

    def security_alert_callback(self, msg):
        self.last_security_alert_msg = msg.data
        self.last_security_alert_time = time.time()
        self.get_logger().info(f'Received /security_alert: {msg.data}')

  #--------------------------------------------------------------------------------------------------------------------------------------------------
    # Helper function:-

    def topic_status(self, last_time, timeout=5.0):
        if last_time is None:
            return 'NO DATA YET'

        elapsed = time.time() - last_time
        if elapsed <= timeout:
            return f'ACTIVE (last update {elapsed:.1f}s ago)'
        else:
            return f'STALE (last update {elapsed:.1f}s ago)'
        
#--------------------------------------------------------------------------------------------------------------------------------------------------
    # Monitor report:-
    
    def print_system_status(self):   #io: def print_system_status(self, event):
        self.get_logger().info('================ SYSTEM STATUS ================')
            #io: rospy.loginfo('================ SYSTEM STATUS ================')

        self.get_logger().info(
            f'/camera_frames      -> {self.topic_status(self.last_camera_time)}'
        )
        self.get_logger().info(
            f'/detected_objects   -> {self.topic_status(self.last_detected_objects_time)}'
        )
        self.get_logger().info(
            f'/object_depth       -> {self.topic_status(self.last_object_depth_time)}'
        )
        self.get_logger().info(
            f'/scene_analysis     -> {self.topic_status(self.last_scene_analysis_time)}'
        )
        self.get_logger().info(
            f'/security_event     -> {self.topic_status(self.last_security_event_time)}'
        )
        self.get_logger().info(
            f'/security_alert     -> {self.topic_status(self.last_security_alert_time)}'
        )

        if self.last_detected_objects_msg is not None:
            self.get_logger().info(
                f'Last detected objects: {self.last_detected_objects_msg}'
            )

        if self.last_object_depth_msg is not None:
            self.get_logger().info(
                f'Last depth value: {self.last_object_depth_msg:.2f} meters'
            )

        if self.last_security_event_msg is not None:
            self.get_logger().info(
                f'Last security event: {self.last_security_event_msg}'
            )

        if self.last_security_alert_msg is not None:
            self.get_logger().info(
                f'Last security alert: {self.last_security_alert_msg}'
            )

        self.get_logger().info('===============================================')


def main(args=None):
    rclpy.init(args=args)            #io: rospy.init_node('system_monitor')
    node = SystemMonitorNode()       #io: node = SystemMonitorNode()
        

    try:
        rclpy.spin(node)        #io: rospy.spin()
    except KeyboardInterrupt:
        node.get_logger().info('System Monitor Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()