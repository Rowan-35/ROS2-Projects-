                                                        
import json     #Json-->Python
import math     #for math calculations (sin,cos,..)
import time     #to add timestamp for the result 
from collections import deque                       #deque: list to store the last 10 results inside it 

import cv2      #to edit the camira photo
import numpy as np          #to deal with arrays and numbers

import rclpy                #ros2 library in python --> (in ROS1:import rospy )
from rclpy.node import Node #import class that names node 
from sensor_msgs.msg import Image   # message type in ROS for camera photoes
from std_msgs.msg import String     #message type use it with topics 
from std_srvs.srv import Trigger    # Service type lama ast2bl msg mn8er input
from cv_bridge import CvBridge      # ROS Image --> openCV image

"""
tst2bl mn l topics :/motion_data
                    /camera_frames
                    /object_data
                    /depth_data

htb3t topic :       /camera_motion
"""

# ── Constants ────────────────────────────────────────────────────────────────
HISTORY_SIZE        = 10            #num of the last results to store
DIRECTION_THRESHOLD = 5.0           #min pixel move
BLUR_VARIANCE_LIMIT = 100.0         #lw a2l mn l kema dy l sora mn8msha w blur
FEATURE_COUNT_MIN   = 10            # a2l 3dd features 3ashan l 7rka tkon mawsoka


class VOEstimationNode(Node):       #b3rf l node k class 3ashan ast5dm l publisher , subscribtion , ....

    def __init__(self):
        super().__init__("vo_estimation_node")      #asm l node : vo_estimation_node


    # ROS1:
    #class VOEstimationNode:
    #def __init__(self):
    #rospy.init_node("vo_estimation_node")

        # ── ROS2 Parameters ──────────────────────────────────────────────────
        self.declare_parameter("focal_length",        500.0)            #self.focal_length = rospy.get_param("~focal_length", 500.0)
        self.declare_parameter("direction_threshold", DIRECTION_THRESHOLD)
        self.declare_parameter("blur_threshold",      BLUR_VARIANCE_LIMIT)
        self.declare_parameter("min_features",        FEATURE_COUNT_MIN)
        self.declare_parameter("use_depth_hint",      True)
        self.declare_parameter("use_object_bias",     True)

        self.focal_length    = self.get_parameter("focal_length").value
        self.dir_threshold   = self.get_parameter("direction_threshold").value
        self.blur_threshold  = self.get_parameter("blur_threshold").value
        self.min_features    = self.get_parameter("min_features").value
        self.use_depth_hint  = self.get_parameter("use_depth_hint").value
        self.use_object_bias = self.get_parameter("use_object_bias").value

        # ── Internal state ───────────────────────────────────────────────────
        self._bridge         = CvBridge()       #ros image ---> opencv
        self._history        = deque(maxlen=HISTORY_SIZE)   # (dx, dy) pairs
        self._frame_count    = 0
        self._latest_motion  = {}    # from Node 4.5
        self._latest_depth   = {}    # from Node 4.3
        self._latest_objects = {}    # from Node 4.2
        self._blur_from_raw  = None  # computed locally from /camera_frames

        # ── Publishers ───────────────────────────────────────────────────────
        self.camera_motion_pub = self.create_publisher(String, "/camera_motion", 10)        #hb3t resala 3ala l topic camera motion 
                                                                                                #self.camera_motion_pub = rospy.Publisher("/camera_motion", String, queue_size=10)

        # ── Subscribers (all 4 upstream sources) ─────────────────────────────
        # hst2bl a5r 10 kym mn l topics dol
        self.create_subscription(String, "/motion_data",            #rospy.Subscriber("/motion_data", String, self._cb_motion, queue_size=10)
                                 self._cb_motion,  10)

        # SECONDARY – Node 4.1 Camera Stream (raw frames for independent blur)
        self.create_subscription(Image, "/camera_frames",
                                 self._cb_camera_frame, 10)

        # SECONDARY – Node 4.2 Object Detection
        self.create_subscription(String, "/object_data",
                                 self._cb_object_data, 10)

        # SECONDARY – Node 4.3 Depth Estimation
        self.create_subscription(String, "/depth_data",
                                 self._cb_depth_data, 10)

        # ── Service /estimate_motion ─────────────────────────────────────────
        self.create_service(Trigger, "/estimate_motion",
                            self._handle_estimate_motion)

        self.get_logger().info(             #rsalt l bdaya
            "VO Node STARTED | focal=%.0fpx thr=%.1fpx blur=%.0f "
            "depth_hint=%s obj_bias=%s"
            % (self.focal_length, self.dir_threshold, self.blur_threshold,
               self.use_depth_hint, self.use_object_bias)
        )

    # ════════════════════════════════════════════════════════════════════════
    # SUBSCRIBER CALLBACKS
    # ════════════════════════════════════════════════════════════════════════

    def _cb_motion(self, msg: String):      #btsht8l automatic lma twsl msg

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:        #lw l resala msh json
            self.get_logger().warn("VO ← /motion_data: bad JSON, skipping") #ytb3 warning       #rospy.loginfo("message")
                                                                                                #rospy.logwarn("message")
            return      #ytl3

        self._latest_motion = data      #y5zn
        self._frame_count  += 1         #yzwd
        result = self._estimate()       #ynady 3la estimate 3ashan t7sb 7rkt l camera
        self._publish(result)           # tnshor l natega 3ala camera motion

    def _cb_camera_frame(self, msg: Image):
        try:
           
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")        #ros->opencv (RGB)
            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)                         #y7wlha gray 3ashan mb7tgsh l alwan f blur
         
            self._blur_from_raw = float(cv2.Laplacian(gray, cv2.CV_64F).var())      # by7sb l blur

            if not self._latest_motion:
      
                self._latest_motion = {"dx": 0.0, "dy": 0.0, "magnitude": 0.0, "feature_count": 20}
            
   
            result = self._estimate()           #ysh8l 7sab l 7rka
            self._publish(result)               #ynshor l natega

        except Exception as e:                  # lw 7sl error
            self.get_logger().warn("Error in camera callback: %s" % str(e))

    def _cb_object_data(self, msg: String):
   
        try:
            self._latest_objects = json.loads(msg.data)     #json-->dictionary
        except json.JSONDecodeError:                        #7sl error
            pass                                            #ytgahl

    def _cb_depth_data(self, msg: String):
  
        try:
            self._latest_depth = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ════════════════════════════════════════════════════════════════════════
    # SERVICE HANDLER                                                                           
    # ════════════════════════════════════════════════════════════════════════

    def _handle_estimate_motion(self, _req, response):
        """
        /estimate_motion  –  Trigger service.
        Any node (e.g. Node 4.7) may call this to get the latest VO result
        synchronously instead of waiting for the next publish.
        """
        if not self._latest_motion:
            response.success = False
            response.message = json.dumps({"error": "No motion data yet"})
            return response
        result           = self._estimate()
        response.success = True
        response.message = json.dumps(result)
        self.get_logger().info(
            "VO /estimate_motion → dir=%s reliable=%s"
            % (result["direction"], result["reliable"])
        )
        return response

    # ════════════════════════════════════════════════════════════════════════
    # CORE ESTIMATION
    # ════════════════════════════════════════════════════════════════════════

    def _estimate(self) -> dict:
        """
        Combines data from all four upstream nodes and returns a camera-motion
        summary dict.
        """
        data = self._latest_motion

        dx            = float(data.get("dx",            0.0))
        dy            = float(data.get("dy",            0.0))
        magnitude     = float(data.get("magnitude",     0.0))
        feature_count = int(  data.get("feature_count", 0))
        timestamp     = float(data.get("timestamp",     time.time()))

        # ── Blur: use OUR own measurement if available, else trust Node 4.5 ──
        blur_variance = (
            self._blur_from_raw
            if self._blur_from_raw is not None
            else float(data.get("blur_variance", 999.0))
        )

        # ── Reliability checks ───────────────────────────────────────────────
        reliable          = True
        unreliable_reason = None

        if blur_variance < self.blur_threshold:
            reliable          = False
            unreliable_reason = "BLURRY_FRAME (var=%.1f)" % blur_variance

        elif feature_count < self.min_features:
            reliable          = False
            unreliable_reason = "LOW_FEATURES (n=%d)" % feature_count

        # Dynamic scene bias from Node 4.2:  >5 objects with spread positions
        # means flow vectors may not reflect camera motion at all.
        if reliable and self.use_object_bias:
            obj_count = self._latest_objects.get("count", 0)
            if obj_count > 5:
                reliable          = False
                unreliable_reason = "DYNAMIC_SCENE (objects=%d)" % obj_count

        # Incoherent large magnitude (camera shake / dynamic background)
        if reliable and magnitude > 80.0 and abs(dx) < 2.0 and abs(dy) < 2.0:
            reliable          = False
            unreliable_reason = "INCOHERENT_FLOW (mag=%.1f but dx≈dy≈0)" % magnitude

        # ── Smoothing ────────────────────────────────────────────────────────
        self._history.append((dx, dy))
        dx_s  = sum(p[0] for p in self._history) / len(self._history)
        dy_s  = sum(p[1] for p in self._history) / len(self._history)
        mag_s = math.hypot(dx_s, dy_s)

        # ── Direction classification ─────────────────────────────────────────
        lateral      = "none"
        longitudinal = "none"
        direction    = "unknown"

        if reliable:
            abs_dx = abs(dx_s)
            abs_dy = abs(dy_s)

            # Lateral pan
            if abs_dx >= self.dir_threshold:
                lateral = "right" if dx_s < 0 else "left"

            # Forward / backward
            if abs_dy >= self.dir_threshold:
                longitudinal = "forward" if dy_s > 0 else "backward"

            # Depth hint from Node 4.3 reinforces longitudinal
            if self.use_depth_hint and self._latest_depth:
                close = self._latest_depth.get("close_obstacle", False)
                if close and longitudinal == "none" and abs_dy > 1.0:
                    longitudinal = "forward"   # approaching obstacle

            # Dominant direction
            if lateral != "none" and longitudinal != "none":
                direction = lateral if abs_dx >= abs_dy else longitudinal
            elif lateral != "none":
                direction = lateral
            elif longitudinal != "none":
                direction = longitudinal
            else:
                direction = "stationary"

        result = {
            "direction":         direction,
            "lateral":           lateral,
            "longitudinal":      longitudinal,
            "reliable":          reliable,
            "unreliable_reason": unreliable_reason,
            "dx_smooth":         round(dx_s, 3),
            "dy_smooth":         round(dy_s, 3),
            "magnitude_smooth":  round(mag_s, 3),
            "blur_variance":     round(blur_variance, 2),
            "feature_count":     feature_count,
            "object_count":      self._latest_objects.get("count", 0),
            "close_obstacle":    self._latest_depth.get("close_obstacle", False),
            "frame_count":       self._frame_count,
            "timestamp":         timestamp,
        }

        if reliable:
            self.get_logger().info(
                "VO → dir=%-12s lat=%-6s lon=%-9s dx=%.2f dy=%.2f mag=%.2f"
                % (direction, lateral, longitudinal, dx_s, dy_s, mag_s)
            )
        else:
            self.get_logger().warn(
                "VO → UNRELIABLE: %s  →  publishing direction=unknown (STOP)"
                % unreliable_reason
            )

        return result

    # ════════════════════════════════════════════════════════════════════════
    # PUBLISH
    # ════════════════════════════════════════════════════════════════════════

    def _publish(self, result: dict):
        msg      = String()
        msg.data = json.dumps(result)
        self.camera_motion_pub.publish(msg)


# ── Entry point ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = VOEstimationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()