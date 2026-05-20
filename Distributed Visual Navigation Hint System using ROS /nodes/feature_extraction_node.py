import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from navigation_msgs.msg import ROIFeatures
import cv2
from cv_bridge import CvBridge
import numpy as np

class ROIFeatureExtractor(Node):
    def __init__(self):
        super().__init__('roi_feature_extractor')
        
        # Parameters
        self.declare_parameter('roi_size', 100)   
        self.declare_parameter('grid_rows', 3)    
        self.declare_parameter('grid_cols', 4)    
        self.declare_parameter('feature_detector', 'orb')  
        self.declare_parameter('max_keypoints', 50)
        
        self.roi_size = self.get_parameter('roi_size').value
        self.grid_rows = self.get_parameter('grid_rows').value
        self.grid_cols = self.get_parameter('grid_cols').value
        self.feature_detector = self.get_parameter('feature_detector').value
        self.max_keypoints = self.get_parameter('max_keypoints').value
        
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            Image,
            '/camera_frames',
            self.image_callback,
            10
        )
        
        self.pub = self.create_publisher(ROIFeatures, '/roi_features', 10)
        
        if self.feature_detector == 'orb':
            self.detector = cv2.ORB_create(nfeatures=self.max_keypoints)
        else:
            self.detector = cv2.GFTTDetector_create(maxCorners=self.max_keypoints)
        
        self.get_logger().info("ROI Feature Extractor Node Started")
    
    def compute_roi_features(self, roi_img):
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        mean_hue = np.mean(hsv[:,:,0])
        mean_sat = np.mean(hsv[:,:,1])
        mean_val = np.mean(hsv[:,:,2])
        
        gray = cv2.cvtColor(roi_img, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        moments = cv2.moments(edges)
        if moments['m00'] != 0:
            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
        else:
            cx = roi_img.shape[1] / 2
            cy = roi_img.shape[0] / 2
        
        keypoints = self.detector.detect(gray, None)
        kp_count = len(keypoints)
        
        return (mean_hue, mean_sat, mean_val, cx, cy, kp_count)
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion error: {e}")
            return
        
        # Create a copy for visualization
        debug_frame = frame.copy()
        
        h, w = frame.shape[:2]
        step_x = w // self.grid_cols
        step_y = h // self.grid_rows
        
        roi_ids, centroids_x, centroids_y = [], [], []
        mean_hue, mean_saturation, mean_value = [], [], []
        keypoint_count = []
        bbox_x, bbox_y, bbox_w, bbox_h = [], [], [], []
        
        roi_index = 0
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                x1, y1 = col * step_x, row * step_y
                x2, y2 = min(x1 + step_x, w), min(y1 + step_y, h)
                
                if x2 - x1 < 10 or y2 - y1 < 10:
                    continue
                
                roi_img = frame[y1:y2, x1:x2]
                (mh, ms, mv, cx, cy, kps) = self.compute_roi_features(roi_img)
                
                # Absolute coordinates for drawing
                abs_cx, abs_cy = int(cx + x1), int(cy + y1)
                
                # --- VISUALIZATION LOGIC ---
                # Draw ROI rectangle
                cv2.rectangle(debug_frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                # Draw centroid
                cv2.circle(debug_frame, (abs_cx, abs_cy), 3, (0, 0, 255), -1)
                # Label the ROI index
                cv2.putText(debug_frame, str(roi_index), (x1+5, y1+15), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                roi_ids.append(roi_index)
                centroids_x.append(float(abs_cx))
                centroids_y.append(float(abs_cy))
                mean_hue.append(float(mh))
                mean_saturation.append(float(ms))
                mean_value.append(float(mv))
                keypoint_count.append(float(kps))
                bbox_x.append(float(x1))
                bbox_y.append(float(y1))
                bbox_w.append(float(x2 - x1))
                bbox_h.append(float(y2 - y1))
                
                roi_index += 1
        
        # Publish features
        roifeatures_msg = ROIFeatures()
        roifeatures_msg.num_rois = len(roi_ids)
        roifeatures_msg.roi_ids = roi_ids
        roifeatures_msg.centroids_x = centroids_x
        roifeatures_msg.centroids_y = centroids_y
        roifeatures_msg.mean_hue = mean_hue
        roifeatures_msg.mean_saturation = mean_saturation
        roifeatures_msg.mean_value = mean_value
        roifeatures_msg.keypoint_count = keypoint_count
        roifeatures_msg.bbox_x = bbox_x
        roifeatures_msg.bbox_y = bbox_y
        roifeatures_msg.bbox_w = bbox_w
        roifeatures_msg.bbox_h = bbox_h
        
        self.pub.publish(roifeatures_msg)

        # --- SHOW WINDOW ---
        cv2.imshow("ROI Feature Extraction", debug_frame)
        cv2.waitKey(1) # Necessary for the window to refresh

def main(args=None):
    rclpy.init(args=args)
    node = ROIFeatureExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()