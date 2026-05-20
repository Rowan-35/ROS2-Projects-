import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2 as cv
from std_msgs.msg import String

class MotionEstimationNode(Node):
    def __init__(self):
        super().__init__('motion_estimation_node')

        self.declare_parameter('focal_length', 525.0)
        f = self.get_parameter('focal_length').get_parameter_value().double_value
        
        self.sub = self.create_subscription(Float32MultiArray,'geometric_inliers',self.callback,10)
        self.sub
        
        self.pub = self.create_publisher(String,'camera_motion',10)

        # Camera intrinsics (you can adjust cx, cy)
        self.K = np.array([[f, 0, 320],
                           [0, f, 240],
                           [0, 0,   1]])

        


    def callback(self, msg):
        data = msg.data
        if len(data) < 10:  # Check "must detect >= 20 keypoints" 
            self.pub.publish(String(data="Insufficient Features"))
            return
        
        points_prev = []
        points_curr = []

        
        for i in range(0, len(data), 4):
            points_prev.append([data[i], data[i+1]])  #(x_prev, y_prev)
            points_curr.append([data[i+2], data[i+3]]) #(x_curr, y_curr)

        # Convert to numpy arrays explicitly for OpenCV
        points_prev = np.array(points_prev, dtype=np.float32)
        points_curr = np.array(points_curr, dtype=np.float32)

        # 1) Compute Essential Matrix
        E, mask = cv.findEssentialMat(points_prev, points_curr, self.K)# type: ignore

        # 2) Recover Pose 
        _, R, t, _ = cv.recoverPose(E, points_prev, points_curr, self.K, mask = mask)# type: ignore

        
        # Determine Directions 
        directions = []
        if t[0] > 0.1: directions.append("Right")
        elif t[0] < -0.1: directions.append("Left")
        
        if t[1] > 0.1: directions.append("Down")
        elif t[1] < -0.1: directions.append("Up")
        
        if t[2] > 0.1: directions.append("Forward")
        elif t[2] < -0.1: directions.append("Backward")
        

        # publish
        # out = Float32MultiArray()
        # out.data = list(R.flatten()) + list(t.flatten())
        # self.pub.publish(out)
        motion_msg = ", ".join(directions) if directions else "Stationary"
        self.get_logger().info(f"Estimated Motion: {motion_msg}")
        self.pub.publish(String(data=motion_msg))

def main(args=None):
    rclpy.init(args=args)
    motion_estimation_node = MotionEstimationNode()
    rclpy.spin(motion_estimation_node)
    motion_estimation_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()




# # #------------------------------without camera intrinsics------------------------------
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, String
# import numpy as np

# class MotionEstimationNode(Node):
#     def __init__(self):
#         super().__init__('motion_estimation_node')

#         self.sub = self.create_subscription(Float32MultiArray,'geometric_inliers',self.callback,10)
#         self.sub
        
#         self.pub = self.create_publisher(String,'camera_motion',10)

#     def callback(self, msg):
#         data = msg.data

#         points_prev = []
#         points_curr = []

#         for i in range(0, len(data), 4):
#             points_prev.append([data[i], data[i+1]])
#             points_curr.append([data[i+2], data[i+3]])

#         points_prev = np.float32(points_prev) # type: ignore
#         points_curr = np.float32(points_curr) # type: ignore

#         flow = points_curr - points_prev
#         avg_flow = np.mean(flow, axis=0)

#         direction = []

#         # Horizontal motion
#         if avg_flow[0] > 1:
#             direction.append("Camera moving LEFT")
#         elif avg_flow[0] < -1:
#             direction.append("Camera moving RIGHT")

#         # Vertical motion
#         if avg_flow[1] > 1:
#             direction.append("Camera moving UP")
#         elif avg_flow[1] < -1:
#             direction.append("Camera moving DOWN")

#         # Forward/backward (approx)
#         dist_prev = np.linalg.norm(points_prev, axis=1)
#         dist_curr = np.linalg.norm(points_curr, axis=1)
#         scale_change = np.mean(dist_curr - dist_prev)

#         if scale_change > 1:
#             direction.append("Camera moving FORWARD")
#         elif scale_change < -1:
#             direction.append("Camera moving BACKWARD")

#         msg_out = String()
#         msg_out.data = ", ".join(direction) if direction else "No significant motion"
#         self.pub.publish(msg_out)

