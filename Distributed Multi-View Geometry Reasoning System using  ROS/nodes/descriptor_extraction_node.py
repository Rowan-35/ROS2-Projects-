import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from multi_view_interfaces.msg import KeypointArray, DescriptorArray, Descriptor
import cv2
from cv_bridge import CvBridge
import message_filters
import numpy as np

class DescriptorExtractionNode(Node):
    def __init__(self):
        super().__init__('descriptor_extraction_node')

        # 1. Parameters
        self.declare_parameter('descriptor_type', 'orb') #BRIEF, BRISK, FREAK or ....
        self.desc_type = self.get_parameter('descriptor_type').get_parameter_value().string_value

        self.bridge = CvBridge()

        # 2. Synchronized Subscribers
        # We must sync /camera_frames and /keypoints so we use the correct image for the detected points
        self.image_sub = message_filters.Subscriber(self, Image, '/camera_frames')
        self.kp_sub = message_filters.Subscriber(self, KeypointArray, '/keypoints')

        # slop=0.1 allows for a 100ms difference between the two messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.kp_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.extraction_process)

        # 3. Publisher
        self.descriptors_publisher = self.create_publisher(DescriptorArray, '/descriptors', 10)

        # 4. Initialize Descriptor Extractor (ORB)
        if self.desc_type.lower() == 'orb':
            self.descriptor_extractor = cv2.ORB_create()
        else:
            self.get_logger().warn(f"Descriptor type {self.desc_type} not supported, using ORB.")
            self.descriptor_extractor = cv2.ORB_create()

        self.get_logger().info(f"Descriptor Extraction Node started. Type: {self.desc_type}")

    def extraction_process(self, image_msg, kp_msg):
        try:
            # A. Convert Image
            frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # B. Convert ROS Keypoints to OpenCV KeyPoints
            cv_keypoints = []
            for kp in kp_msg.keypoints_array:
                cv_kp = cv2.KeyPoint(
                    x=float(kp.x),
                    y=float(kp.y),
                    size=float(kp.size),
                    angle=float(kp.angle),
                    response=float(kp.response),
                    octave=int(kp.octave),
                    class_id=int(kp.class_id)
                )
                cv_keypoints.append(cv_kp)

            if not cv_keypoints:
                self.get_logger().debug("No keypoints received in this frame.")
                return

            # C. Compute Descriptors
            # The .compute() method returns (keypoints, descriptors)
            # 'descriptors' is an Nx32 numpy array for ORB
            _, descriptors = self.descriptor_extractor.compute(gray, cv_keypoints)

            if descriptors is not None:
                # D. Create the main Array Message
                out_msg = DescriptorArray()
                out_msg.header = image_msg.header  # Carry over Frame ID and timestamp
                out_msg.descriptor_length = int(descriptors.shape[1])

                # E. Wrap each row into a Descriptor object (Prevents Assertion Error)
                for row in descriptors:
                    desc_obj = Descriptor()
                    desc_obj.data = row.tolist() # uint8 list
                    out_msg.descriptors.append(desc_obj)

                # F. Publish
                self.descriptors_publisher.publish(out_msg)
                self.get_logger().info(f"Published {len(descriptors)} descriptors for Frame ID: {image_msg.header.frame_id}")

        except Exception as e:
            self.get_logger().error(f"Failed to extract descriptors: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DescriptorExtractionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()