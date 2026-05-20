import sys
import rclpy
import cv2 
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStreamingNode(Node):
    def __init__(self, stream_source):
        super().__init__('camera_stream_node')
        # 1. Publishers and variables
        self.frames_publisher = self.create_publisher(Image, '/camera_frames', 10)
        self.bridge = CvBridge()
        self.frame_id = 0

        # 2. Stream sources definition (Fixed indentation and added backends)
        if stream_source == 1:
            self.get_logger().info("Webcam is being used")
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        elif stream_source == 2:
            self.get_logger().info("Video 1 is being used")
            self.cap = cv2.VideoCapture("/home/nada/Downloads/traffic1.mp4", cv2.CAP_FFMPEG)
        elif stream_source == 3:
            self.get_logger().info("Video 2 is being used")
            self.cap = cv2.VideoCapture("/home/nada/Downloads/traffic2.mp4", cv2.CAP_FFMPEG)
        else:
            self.get_logger().info("Video 3 is being used")
            self.cap = cv2.VideoCapture("/home/nada/Downloads/traffic3.mp4", cv2.CAP_FFMPEG)
        
        # 3. Timer
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            # Fixed: added () to get_logger
            self.get_logger().warning("End of video or camera error")
            return

        # --- Show Video Window ---
        cv2.imshow("Camera Stream", frame)
        cv2.waitKey(1) 

        # --- Print Frame ID ---
        print(f"Publishing Frame ID: {self.frame_id}")

        # Convert image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        
        # Add metadata
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_id)

        # Increase counter and publish
        self.frame_id += 1
        self.frames_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run smart_security_system camera_stream_node <option>")
        print("1 = Webcam")
        print("2 = Video1")
        print("3 = Video2")
        print("4 = Video3")
        sys.exit(1)

    source_option = int(sys.argv[1])
    node = CameraStreamingNode(source_option)

    rclpy.spin(node)

    # Cleanup
    node.cap.release()
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
