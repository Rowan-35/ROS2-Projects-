import sys
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraStreamNode(Node):
    def __init__(self, stream_source_arg):
        super().__init__('camera_stream_node')
        # 1. declaring variable and stream sources path 
        self.frames_publisher = self.create_publisher(Image, '/camera_frames', 10) #publish through ?camera_frames topic
        self.bridge = CvBridge()
        self.stream_path = "0" # Initialize as string "0" to keep parameter types consistent
        self.frame_id=0
        if stream_source_arg == 1:
            self.stream_path = "/home/nada/Downloads/road1.mp4"
        elif stream_source_arg == 2:
            self.stream_path= "/home/nada/Downloads/road2.mp4"
        elif stream_source_arg == 3:
            self.stream_path= "/home/nada/Downloads/road3.mp4"
        elif stream_source_arg == 4:
            self.stream_path= "/home/nada/Downloads/road4.mp4" 
        #elif stream_source_arg == 5:
            #self.stream_path= "/home/nada/Downloads/road5.mp4"
        else:
            self.stream_path= "0"
        
        # 2. declaring parameters (frame _rate & stream_source)
        self.declare_parameter('stream_source', self.stream_path)
        self.declare_parameter('frame_rate', 10) # Minimum is 5 FPS 
        
        # Accessing values using .value for ROS 2 compatibility
        self.conf_frame_rate = self.get_parameter('frame_rate').value
        self.conf_stream_source = self.get_parameter('stream_source').value

        # 3. Stream logic
        # Convert "0" to integer 0 for webcam, otherwise use the file path string
        final_source = 0 if self.conf_stream_source == "0" else self.conf_stream_source
        self.cap = cv2.VideoCapture(final_source)

        # 4. Timer creation
        self.timer = self.create_timer(1.0 / self.conf_frame_rate, self.timer_callback)
    
    def timer_callback(self):
        ret, frame= self.cap.read()
        if not ret:
            self.get_logger().warning("End of video or camera error")
            self.get_logger().info("Stream finished. Closing...")
            cv2.destroyAllWindows() # Closes the 'camera stream' window [cite: 45]
            self.timer.cancel()
            return


        #visualization
        cv2.namedWindow("camera stream", cv2.WINDOW_NORMAL)
        cv2.imshow("camera stream",frame)
        cv2.waitKey(1)
        print(f"Publishing Frame ID: {self.frame_id}")

        # _________Main Function type conversion____________
        msg= self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = str(self.frame_id)
        self.frame_id += 1
        #publish 
        self.frames_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run smart_security_system camera_stream_node <option>")
        print("0 = Webcam")
        print("1 = Video1")
        print("2 = Video2")
        print("3 = Video3")
        print("4 = Video4")
        print("5 = Video5")
        sys.exit(1)

    source_option = int(sys.argv[1])
    node = CameraStreamNode(source_option)

    rclpy.spin(node)

    # Cleanup
    node.cap.release()
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
