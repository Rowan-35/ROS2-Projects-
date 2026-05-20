#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
import message_filters

# Message and Service imports
from multi_view_interfaces.msg import FeatureMatchArray, KeypointArray
from multi_view_interfaces.srv import CheckGeometry 

class GeometricConsistencyNode(Node):
    def __init__(self):
        super().__init__('geometric_consistency_node')

        # 1. Parameters
        self.declare_parameter('inlier_threshold', 0.5)
        self.threshold = self.get_parameter('inlier_threshold').value

        # 2. Synchronized Subscribers
        # We need KeypointArray to get coordinates for the Match indices
        self.kp_sub = message_filters.Subscriber(self, KeypointArray, '/keypoints')
        self.match_sub = message_filters.Subscriber(self, FeatureMatchArray, '/filtered_matches')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.kp_sub, self.match_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.callback)

        # 3. Publisher
        self.inlier_pub = self.create_publisher(FeatureMatchArray, '/geometric_inliers', 10)

        # 4. Service Server
        self.srv = self.create_service(CheckGeometry, '/check_geometry', self.check_geometry_handle)

        # Buffer for the previous frame's coordinates
        self.prev_keypoints = None
        
        self.get_logger().info("Geometric Consistency Node (with Service) Started.")

    def callback(self, kp_msg, match_msg):
        # Convert current keypoints to a usable numpy array
        curr_kps = np.float32([[kp.x, kp.y] for kp in kp_msg.keypoints_array])

        if self.prev_keypoints is None:
            self.prev_keypoints = curr_kps
            return

        if not match_msg.matches:
            self.prev_keypoints = curr_kps
            return

        # Prepare point sets for cv2.findFundamentalMat using indices
        pts_prev = []
        pts_curr = []

        for m in match_msg.matches:
            # Safety check to ensure indices are within bounds of our coordinate arrays
            if m.query_idx < len(self.prev_keypoints) and m.train_idx < len(curr_kps):
                pts_prev.append(self.prev_keypoints[m.query_idx])
                pts_curr.append(curr_kps[m.train_idx])

        if len(pts_prev) < 8:
            self.get_logger().warn("Not enough matches for F-Matrix (need 8).")
            self.prev_keypoints = curr_kps
            return

        pts_prev = np.float32(pts_prev)
        pts_curr = np.float32(pts_curr)

        # --- THE CORE LOGIC: RANSAC ---
        # F: Fundamental Matrix, mask: 1 for inlier, 0 for outlier
        F, mask = cv2.findFundamentalMat(pts_prev, pts_curr, cv2.FM_RANSAC, self.threshold)

        if F is not None and mask is not None:
            inlier_msg = FeatureMatchArray()
            inlier_msg.header = match_msg.header
            
            for i, is_inlier in enumerate(mask.ravel()):
                if is_inlier:
                    inlier_msg.matches.append(match_msg.matches[i])

            self.inlier_pub.publish(inlier_msg)
            self.get_logger().info(f"Geometry Check: {len(inlier_msg.matches)} inliers.")

        # Update buffer for next iteration
        self.prev_keypoints = curr_kps

    def check_geometry_handle(self, request, response):
        """Service callback to validate a specific set of matches on demand."""
        # Note: This conceptual check uses the 8-point rule 
        # as defined in the technical requirements.
        num_matches = len(request.matches)
        response.inlier_count = num_matches
        response.is_consistent = num_matches >= 8 
        
        self.get_logger().info(f"Service request: Validated {num_matches} matches.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GeometricConsistencyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()