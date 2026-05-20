#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge    
import torch
import numpy as np
import os
import sys
import cv2  

# --- DYNAMIC PATH SETUP ---
HOME = os.path.expanduser("~")
REPO_PATH = os.path.join(HOME, "Depth-Anything-V2")
sys.path.append(REPO_PATH)

from depth_anything_v2.dpt import DepthAnythingV2

class DepthEstimationNode(Node):
    def __init__(self):
        super().__init__('DepthEstimationNode') 

        self.declare_parameter('depth_model_path', 'vits') 
        encoder = self.get_parameter('depth_model_path').get_parameter_value().string_value 

        # --------------------- Model Setup -----------------------
        DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        model_configs = {
            'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
            'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
            'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
            'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
        }

        self.model = DepthAnythingV2(**model_configs[encoder])
        
        ckpt_name = f'depth_anything_v2_{encoder}.pth'
        full_ckpt_path = os.path.join(REPO_PATH, 'checkpoints', ckpt_name)
        
        self.get_logger().info(f'Loading model from: {full_ckpt_path}')
        
        self.model.load_state_dict(torch.load(full_ckpt_path, weights_only=True, map_location='cpu'))
        self.model = self.model.to(DEVICE).eval()
        # ---------------------------------------------------------

        self.subscription = self.create_subscription(Image, 'camera_frames', self.callback_function, 10)
        self.publisher = self.create_publisher(Image, 'object_depth', 10)
        self.bridge = CvBridge()

    def callback_function(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        depth = self.model.infer_image(img) 

        # --- VISUALIZATION ADDITION ---
        # Normalize depth to 0-255 for display
        depth_vis = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = depth_vis.astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_INFERNO) # Optional: Makes it colorful
        
        cv2.imshow("Depth Anything V2 Live", depth_vis)
        cv2.waitKey(1)
        # ------------------------------
        
        depth_raw = np.clip(depth, 0, 10).astype(np.float32)
        ros_image = self.bridge.cv2_to_imgmsg(depth_raw, encoding="32FC1")
        self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    cv2.destroyAllWindows() # <--- Added for cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()