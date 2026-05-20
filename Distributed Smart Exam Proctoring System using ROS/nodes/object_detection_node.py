import rclpy
import cv2 
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_node')
        
        # Parameter
        self.declare_parameter('confidence_threshold', 0.3)
        self.conf_threshold = self.get_parameter(
            'confidence_threshold').get_parameter_value().double_value

        self.bridge = CvBridge()
        
        # Single YOLO model
        self.model = YOLO('yolov8n.pt')
        
        self.subscription = self.create_subscription(
            Image, '/camera_frames', self.detection_process, 10)
        
        self.objects_publisher = self.create_publisher(
            String, '/object_data', 10)

    def detection_process(self, msg):
        cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detections = []

        max_area = 100000

        # YOLO detection (phones + books only)
        results = self.model(cv_frame, conf=self.conf_threshold, classes=[67, 73])

        for box in results[0].boxes:
            b = box.xyxy[0].tolist()

            # Area filter
            if (b[2] - b[0]) * (b[3] - b[1]) > max_area:
                continue

            conf = box.conf[0]
            cls = int(box.cls[0])

            label = "cell phone" if cls == 67 else "book"

            # Draw box (green only)
            cv2.rectangle(cv_frame,
                          (int(b[0]), int(b[1])),
                          (int(b[2]), int(b[3])),
                          (0, 255, 0), 2)

            cv2.putText(cv_frame,
                        f"{label} {conf:.2f}",
                        (int(b[0]), int(b[1] - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0), 2)

            detections.append(
                f"{label},{b[0]:.1f},{b[1]:.1f},{b[2]:.1f},{b[3]:.1f}"
            )

        # Publish
        out_msg = String()
        out_msg.data = "|".join(detections) if detections else "None"
        self.objects_publisher.publish(out_msg)

        # Display
        cv2.namedWindow("Object Detection Feed", cv2.WINDOW_NORMAL)
        cv2.imshow("Object Detection Feed", cv_frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()