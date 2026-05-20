import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge    
import torch
import numpy as np

import sys
sys.path.append("/home/razan/Depth-Anything-V2")
from depth_anything_v2.dpt import DepthAnythingV2



class DepthEstimationNode (Node):
    def __init__(self):
        super().__init__('DepthEstimationNode') 

        self.declare_parameter('depth_model_path', 'vits') # or 'vits', 'vitb', 'vitg'
        encoder = self.get_parameter('depth_model_path').get_parameter_value().string_value 

        # --------------------- Model Setup -----------------------
        DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
        model_configs = {
        'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
        'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
        'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
        'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }
        # encoder = 'vits' # or 'vits', 'vitb', 'vitg'
        self.model = DepthAnythingV2(**model_configs[encoder])
        self.model.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{encoder}.pth', weights_only=True ,map_location='cpu'))
        self.model = self.model.to(DEVICE).eval()
        # ---------------------------------------------------------


        self.subscribe = self.create_subscription(Image, 'camera_frames', self.callback_function, 10)
        self.subscribe  # prevent unused variable warning ???????
        self.publisher = self.create_publisher(Image ,'object_depth', 10 )
        
        self.bridge = CvBridge()

    def callback_function (self,msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        depth = self.model.infer_image(img) # HxW raw depth map in numpy
        
         # Invert depth valies and clip to max depth
        depth = np.clip(depth, 0, 10)
        depth = depth.astype(np.float32)

        # Convert OpenCV Images to ROS Image and publish it
        ros_image = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1") #a depth map is not a 3-channel color image (BGR); it is a 1-channel array of numbers (distances).  #encoding="32FC1" (32-bit Float, 1 Channel).
        self.publisher.publish(ros_image)
       



def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()