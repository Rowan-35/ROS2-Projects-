import sys
import rclpy
import cv2 
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        # 1. declare parameters then get their values
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('model_path', 'yolov8n.pt')

        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # 2. Initialize CV Bridge and YOLO objects
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)

        # 3. Subscriber 
        self.subscription = self.create_subscription(
            Image,
            '/camera_frames',
            self.detection_process,
            10)

        # 4. Publisher
        self.objects_publisher = self.create_publisher(String, '/detected_objects', 10)


    def detection_process(self, msg):
        #Runs after recieving new frame

        frame_id = msg.header.frame_id

        cv_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv8 Inference
        results = self.model(cv_frame, conf=self.conf_threshold)
        detections = []
        for box in results[0].boxes:
            cls = int(box.cls[0])
            name = self.model.names[cls]
            coords = box.xyxy[0].tolist() # [x1, y1, x2, y2]
            detections.append(f"{name},{coords[0]:.1f},{coords[1]:.1f},{coords[2]:.1f},{coords[3]:.1f}")

        self.get_logger().info(f"Received Frame ID: {frame_id} for detection")

    
        # Publish the data
        out_msg = String()
        out_msg.data = "|".join(detections) if detections else "None"
        self.objects_publisher.publish(out_msg)

        # Show visual feedback
        annotated_frame = results[0].plot()
        cv2.imshow("YOLOv8 Security Feed", annotated_frame)
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