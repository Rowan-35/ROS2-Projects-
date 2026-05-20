
import rclpy
from rclpy.node import Node

from multi_view_interfaces.msg import FeatureMatchArray


class MatchFilteringNode(Node):
    def __init__(self):
        super().__init__('match_filtering_node')

        # ============================================================
        # 1. Parameters
        # ============================================================
        # Common value for Lowe's ratio test is 0.75.
        self.declare_parameter('ratio_test_threshold', 0.75)

        # Used only if the raw match does not contain second_distance.
        # Lower value = stricter filtering.
        self.declare_parameter('max_match_distance', 64.0)

        # If True, the node will keep only one best match for each keypoint.
        self.declare_parameter('enforce_cross_check', True)

        # Minimum number of good matches expected.
        self.declare_parameter('min_filtered_matches', 8)

        self.ratio_test_threshold = (
            self.get_parameter('ratio_test_threshold')
            .get_parameter_value()
            .double_value
        )

        self.max_match_distance = (
            self.get_parameter('max_match_distance')
            .get_parameter_value()
            .double_value
        )

        self.enforce_cross_check = (
            self.get_parameter('enforce_cross_check')
            .get_parameter_value()
            .bool_value
        )

        self.min_filtered_matches = (
            self.get_parameter('min_filtered_matches')
            .get_parameter_value()
            .integer_value
        )

        # ============================================================
        # 2. Subscriber
        # ============================================================
        # This node receives the weak/raw matches from Feature Matching Node.
        self.raw_matches_subscriber = self.create_subscription(
            FeatureMatchArray,
            '/raw_matches',
            self.raw_matches_callback,
            10
        )

        # ============================================================
        # 3. Publisher
        # ============================================================
        # This node publishes only the good/filtered matches.
        self.filtered_matches_publisher = self.create_publisher(
            FeatureMatchArray,
            '/filtered_matches',
            10
        )

        self.get_logger().info(
            'Match Filtering Node started. '
            f'ratio_test_threshold={self.ratio_test_threshold}, '
            f'max_match_distance={self.max_match_distance}, '
            f'enforce_cross_check={self.enforce_cross_check}'
        )

    # ============================================================
    # Helper function: safely get a field from a ROS message
    # ============================================================
    def get_msg_field(self, msg, possible_names, default_value=None):
        """
        This function helps if your Match message uses slightly different names.

        Example:
        Maybe your match field is called query_idx.
        Maybe it is called query_index.
        This function checks many possible names.
        """

        for name in possible_names:
            if hasattr(msg, name):
                return getattr(msg, name)

        return default_value

    # ============================================================
    # Main callback
    # ============================================================
    def raw_matches_callback(self, msg):
        """
        This function runs every time the node receives data from /raw_matches.
        """

        # ============================================================
        # 1. Get the raw matches list
        # ============================================================
        # I assume your FeatureMatchArray has a field called matches.
        # If your field name is raw_matches, matches_array, or another name,
        if hasattr(msg, 'matches'):
            raw_matches = list(msg.matches)
        elif hasattr(msg, 'raw_matches'):
            raw_matches = list(msg.raw_matches)
        elif hasattr(msg, 'matches_array'):
            raw_matches = list(msg.matches_array)
        else:
            self.get_logger().error(
                'Could not find matches list in FeatureMatchArray message. '
                'Expected field: matches, raw_matches, or matches_array.'
            )
            return

        if len(raw_matches) == 0:
            self.get_logger().warn('Received empty raw matches message.')
            self.publish_filtered_matches(msg, [])
            return

        # ============================================================
        # 2. Apply ratio test / distance filtering
        # ============================================================
        ratio_filtered_matches = []

        for match in raw_matches:
            distance = self.get_msg_field(
                match,
                ['distance', 'match_distance', 'best_distance'],
                None
            )

            second_distance = self.get_msg_field(
                match,
                ['second_distance', 'second_best_distance', 'second_match_distance'],
                None
            )

            if distance is None:
                self.get_logger().warn(
                    'A match does not contain distance field. Skipping it.'
                )
                continue

            # ------------------------------------------------------------
            # Case 1:
            # The feature matching node published best and second-best distance.
            # This is the real ratio test:
            #
            # distance / second_distance < ratio_test_threshold
            # ------------------------------------------------------------
            if second_distance is not None and second_distance > 0:
                ratio_value = float(distance) / float(second_distance)

                if ratio_value < self.ratio_test_threshold:
                    ratio_filtered_matches.append(match)

            # ------------------------------------------------------------
            # Case 2:
            # The feature matching node published only one distance.
            # We cannot do true ratio test, so we use max_match_distance.
            # ------------------------------------------------------------
            else:
                if float(distance) <= self.max_match_distance:
                    ratio_filtered_matches.append(match)

        # ============================================================
        # 3. Enforce cross-check / one-to-one matching
        # ============================================================
        if self.enforce_cross_check:
            final_matches = self.apply_one_to_one_filter(ratio_filtered_matches)
        else:
            final_matches = ratio_filtered_matches

        # ============================================================
        # 4. Publish result
        # ============================================================
        self.publish_filtered_matches(msg, final_matches)

        # ============================================================
        # 5. Logging
        # ============================================================
        if len(final_matches) < self.min_filtered_matches:
            self.get_logger().warn(
                f'Low number of filtered matches: {len(final_matches)}. '
                f'Minimum expected: {self.min_filtered_matches}. '
                'System may be unreliable.'
            )
        else:
            self.get_logger().info(
                f'Raw matches: {len(raw_matches)} | '
                f'After ratio test: {len(ratio_filtered_matches)} | '
                f'Final filtered matches: {len(final_matches)}'
            )

    # ============================================================
    # One-to-one / cross-check-like filtering
    # ============================================================
    def apply_one_to_one_filter(self, matches):
        """
        This keeps only the best match for every query keypoint and every train keypoint.

        Why?
        Because one keypoint in frame 1 should not match many keypoints in frame 2.
        This is a practical cross-check-like filter.
        """

        best_for_query = {}

        # First pass:
        # keep the lowest-distance match for every query_idx
        for match in matches:
            query_idx = self.get_msg_field(
                match,
                ['query_idx', 'query_index', 'source_idx', 'source_index'],
                None
            )

            distance = self.get_msg_field(
                match,
                ['distance', 'match_distance', 'best_distance'],
                None
            )

            if query_idx is None or distance is None:
                continue

            query_idx = int(query_idx)
            distance = float(distance)

            if query_idx not in best_for_query:
                best_for_query[query_idx] = match
            else:
                old_distance = float(
                    self.get_msg_field(
                        best_for_query[query_idx],
                        ['distance', 'match_distance', 'best_distance'],
                        999999.0
                    )
                )

                if distance < old_distance:
                    best_for_query[query_idx] = match

        query_unique_matches = list(best_for_query.values())

        best_for_train = {}

        # Second pass:
        # keep the lowest-distance match for every train_idx
        for match in query_unique_matches:
            train_idx = self.get_msg_field(
                match,
                ['train_idx', 'train_index', 'target_idx', 'target_index'],
                None
            )

            distance = self.get_msg_field(
                match,
                ['distance', 'match_distance', 'best_distance'],
                None
            )

            if train_idx is None or distance is None:
                continue

            train_idx = int(train_idx)
            distance = float(distance)

            if train_idx not in best_for_train:
                best_for_train[train_idx] = match
            else:
                old_distance = float(
                    self.get_msg_field(
                        best_for_train[train_idx],
                        ['distance', 'match_distance', 'best_distance'],
                        999999.0
                    )
                )

                if distance < old_distance:
                    best_for_train[train_idx] = match

        return list(best_for_train.values())

    # ============================================================
    # Publish filtered matches
    # ============================================================
    def publish_filtered_matches(self, original_msg, filtered_matches):
        """
        Creates a new FeatureMatchArray message and publishes it.
        """

        out_msg = FeatureMatchArray()

        # Copy header if your FeatureMatchArray has header
        if hasattr(out_msg, 'header') and hasattr(original_msg, 'header'):
            out_msg.header = original_msg.header
        elif hasattr(out_msg, 'header'):
            out_msg.header.stamp = self.get_clock().now().to_msg()
            out_msg.header.frame_id = 'filtered_matches'

        # CHANGE HIGHLIGHT:
        # I assume the output message field is called matches.
        # If your FeatureMatchArray field is called filtered_matches or matches_array,
        # change this part.
        if hasattr(out_msg, 'matches'):
            out_msg.matches = filtered_matches
        elif hasattr(out_msg, 'filtered_matches'):
            out_msg.filtered_matches = filtered_matches
        elif hasattr(out_msg, 'matches_array'):
            out_msg.matches_array = filtered_matches
        else:
            self.get_logger().error(
                'Could not publish. FeatureMatchArray has no matches field.'
            )
            return

        self.filtered_matches_publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)

    node = MatchFilteringNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Match Filtering Node stopped by user.')

    finally:
        # Destroy the node first.
        node.destroy_node()

        # Check if ROS is still running before calling shutdown.
        # This prevents:
        # RCLError: rcl_shutdown already called
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
