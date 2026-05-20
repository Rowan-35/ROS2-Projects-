#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import os
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String  
from cv_bridge import CvBridge
import json 

class FaceDetectionFinalNode(Node):
    def __init__(self):
        super().__init__('face_detection_node')

        # Parameters
        self.declare_parameter('scale_factor1', 1.07)
        self.scale_factor1 = self.get_parameter('scale_factor1').get_parameter_value().double_value
        self.declare_parameter('min_neighbors1', 6)
        self.min_neighbors1 = int(self.get_parameter('min_neighbors1').value)
        self.declare_parameter('scale_factor2', 1.04)
        self.scale_factor2 = self.get_parameter('scale_factor2').get_parameter_value().double_value
        self.declare_parameter('min_neighbors2', 6)
        self.min_neighbors2 = int(self.get_parameter('min_neighbors2').value)

        self._bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera_frames',
            self.detection_process,
            10)

        self._preview_pub = self.create_publisher(Image, '/face_data', 10)
        self._coords_pub = self.create_publisher(String, '/face_coordinates', 10)

        base_path = '/usr/share/opencv4/haarcascades/'
        self._front_cascade = cv2.CascadeClassifier(base_path + 'haarcascade_frontalface_alt2.xml')
        self._profile_cascade = cv2.CascadeClassifier(base_path + 'haarcascade_profileface.xml')

        self.get_logger().info("Face Detection Node started.")

    def is_overlapping(self, x, y, w, h, faces):
        for (fx, fy, fw, fh) in faces:
            if abs(x - fx) < w * 0.5 and abs(y - fy) < h * 0.5:
                return True
        return False

    def detection_process(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        front_faces = self._front_cascade.detectMultiScale(
            gray, scaleFactor=self.scale_factor1, minNeighbors=self.min_neighbors1, minSize=(30, 30)
        )
        
        profile_faces = self._profile_cascade.detectMultiScale(
            gray, scaleFactor=self.scale_factor2, minNeighbors=self.min_neighbors2, minSize=(30, 30)
        )

        detected_faces_data = []

        # Frontal faces
        for (x, y, w, h) in front_faces:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, 'Front Face', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            detected_faces_data.append({"type": "front", "x": int(x), "y": int(y), "w": int(w), "h": int(h)})

        # Profile faces
        for (x, y, w, h) in profile_faces:
            if not self.is_overlapping(x, y, w, h, front_faces):
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(frame, 'Side Face', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                detected_faces_data.append({"type": "side", "x": int(x), "y": int(y), "w": int(w), "h": int(h)})

        out_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self._preview_pub.publish(out_msg)

        if detected_faces_data:
            coords_msg = String()
            coords_msg.data = json.dumps(detected_faces_data)
            self._coords_pub.publish(coords_msg)

        cv2.imshow("Face Detection Monitor", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectionFinalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()