#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np
import json


class FeatureMatchingNode(Node):

    def __init__(self):
        super().__init__('feature_matching_node')

        # ── Parameter ────────────────────────────────────────────────
        self.declare_parameter('match_threshold', 0.75)
        self.match_threshold = self.get_parameter('match_threshold').value

        # ── Distance metric: Hamming → correct for ORB binary descriptors
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # ── Buffer: store previous frame descriptors & keypoints ─────
        self.prev_keypoints   = None   # list of [x, y]
        self.prev_descriptors = None   # numpy uint8 array

        # ── ROS Communication ────────────────────────────────────────
        self.sub = self.create_subscription(
            String,
            '/descriptors',
            self.descriptor_callback,
            10
        )

        self.pub = self.create_publisher(
            String,
            '/raw_matches',
            10
        )

        self.get_logger().info(
            '============================================\n'
            '  Node 4.4 — Feature Matching Node STARTED\n'
            f'  match_threshold : {self.match_threshold}\n'
            '  Subscribed      : /descriptors\n'
            '  Publishing      : /raw_matches\n'
            '  Distance metric : Hamming (BFMatcher)\n'
            '============================================'
        )

    # ================================================================
    #   STEP 1 — Receive descriptors from Node 4.3
    # ================================================================
    def descriptor_callback(self, msg: String):

        try:
            data = json.loads(msg.data)

            curr_keypoints   = data['keypoints']     # list of [x, y]
            curr_desc_list   = data['descriptors']   # list of lists (uint8)
            count            = data['count']

            # Guard: need descriptors to match
            if count == 0 or not curr_desc_list:
                self.get_logger().warn('No descriptors received — skipping frame')
                self._update_buffer(curr_keypoints, None)
                return

            # Convert to numpy uint8 — required by cv2 BFMatcher
            curr_desc = np.array(curr_desc_list, dtype=np.uint8)

            # First frame: nothing to match against yet
            if self.prev_descriptors is None:
                self.get_logger().info(
                    f'First frame buffered | {count} descriptors stored')
                self._update_buffer(curr_keypoints, curr_desc)
                return

            # STEP 2 — Compare descriptors & generate matches
            matches = self._compare_and_match(
                self.prev_descriptors,
                curr_desc
            )

            # STEP 3 — Build and publish /raw_matches
            self._publish_matches(
                matches,
                self.prev_keypoints,
                curr_keypoints
            )

        except Exception as e:
            self.get_logger().error(f'Feature matching error: {e}')

        finally:
            # Always advance the buffer to the current frame
            try:
                self._update_buffer(curr_keypoints, curr_desc)
            except Exception:
                pass

    # ================================================================
    #   STEP 2 — Compare descriptors / Generate matches
    #            Distance metric: Hamming via BFMatcher + ratio test
    # ================================================================
    def _compare_and_match(self,
                           desc_prev: np.ndarray,
                           desc_curr: np.ndarray) -> list:
        """
        Compare descriptors between two consecutive frames.

        Strategy
        --------
        BFMatcher     → brute-force comparison of every descriptor pair
        NORM_HAMMING  → distance metric for binary (ORB) descriptors
        knnMatch(k=2) → find 2 nearest neighbours per descriptor
        Lowe ratio    → keep match only if:
                          distance(best) < match_threshold x distance(2nd best)
                        Filters ambiguous matches using the distance metric.
        """

        # Need at least 2 descriptors in each frame for knnMatch k=2
        if len(desc_prev) < 2 or len(desc_curr) < 2:
            raw = self.matcher.match(desc_prev, desc_curr)
            self.get_logger().warn(
                'Too few descriptors for knn — using plain match()')
            return sorted(raw, key=lambda m: m.distance)

        # knnMatch: for each descriptor in prev, find 2 closest in curr
        knn_matches = self.matcher.knnMatch(desc_prev, desc_curr, k=2)

        # Lowe ratio test — use distance metric to filter bad matches
        good_matches = []
        for pair in knn_matches:
            if len(pair) == 2:
                best, second = pair
                # Accept if best distance is significantly smaller than second
                if best.distance < self.match_threshold * second.distance:
                    good_matches.append(best)
            elif len(pair) == 1:
                good_matches.append(pair[0])

        # Sort by distance (best matches first)
        return sorted(good_matches, key=lambda m: m.distance)

    # ================================================================
    #   STEP 3 — Publish /raw_matches
    # ================================================================
    def _publish_matches(self,
                         matches: list,
                         kp_prev: list,
                         kp_curr: list):

        if not matches:
            self.get_logger().warn('No matches passed ratio test')
            return

        # Compute distance statistics
        distances  = [m.distance for m in matches]
        avg_dist   = float(np.mean(distances))
        min_dist   = float(np.min(distances))
        max_dist   = float(np.max(distances))

        # Serialize each match
        match_list = []
        for m in matches:
            prev_pt = kp_prev[m.queryIdx] if m.queryIdx < len(kp_prev) else [0.0, 0.0]
            curr_pt = kp_curr[m.trainIdx] if m.trainIdx < len(kp_curr) else [0.0, 0.0]
            match_list.append({
                'queryIdx' : m.queryIdx,   # index in previous frame
                'trainIdx' : m.trainIdx,   # index in current frame
                'distance' : float(m.distance),
                'prev_pt'  : prev_pt,      # [x, y] in previous frame
                'curr_pt'  : curr_pt,      # [x, y] in current frame
            })

        payload = json.dumps({
            'match_count'  : len(match_list),
            'avg_distance' : avg_dist,
            'min_distance' : min_dist,
            'max_distance' : max_dist,
            'threshold'    : self.match_threshold,
            'matches'      : match_list,
        })

        self.pub.publish(String(data=payload))

        self.get_logger().info(
            f'Published /raw_matches | '
            f'count={len(match_list)} | '
            f'avg_dist={avg_dist:.1f} | '
            f'min={min_dist:.1f} | '
            f'max={max_dist:.1f}'
        )

    # ================================================================
    #   Buffer management
    # ================================================================
    def _update_buffer(self, keypoints, descriptors_np):
        self.prev_keypoints   = keypoints
        self.prev_descriptors = descriptors_np


# ────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FeatureMatchingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
