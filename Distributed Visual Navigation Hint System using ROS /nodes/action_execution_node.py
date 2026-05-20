import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.action import ActionServer
from action_pkg.action import Navigate
import time


class ActionExecutionNode (Node):

    def __init__(self):
        super().__init__('ActionExecutionNode')
        
        self.command = "stop"

        self._action_server = ActionServer(self, Navigate, 'navigate_action', self.execute_callback)

        self.declare_parameter('action_duration', 2.0) 
        self.duration = self.get_parameter('action_duration').get_parameter_value().double_value

        self.subscribe = self.create_subscription(String , 'navigation_command', self.callback_function, 10)
        self.subscribe 
        
        self.publisher = self.create_publisher(String ,'action_status', 10 )

   



    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Navigate.Feedback()
        
        start_time = time.time()
        while (time.time() - start_time) < self.duration:
            if  self.command == "move left":
                self.publisher.publish(String(data="status : Moving left"))
            elif self.command == "move right":
                self.publisher.publish(String(data="status : Moving right"))
            elif self.command == "stop":
                self.publisher.publish(String(data="status : Stopping"))
            
            feedback_msg = Navigate.Feedback()
            feedback_msg.time = [time.time() - start_time]
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.1) 


        goal_handle.succeed()
        
        result = Navigate.Result("Action Finished Successfully")
        return result



    def callback_function (self,msg) :
        self.command = msg.data.lower()
            






def main(args=None):
    rclpy.init(args=args)
    navigate_action_server = ActionExecutionNode()
    rclpy.spin(navigate_action_server)

if __name__ == '__main__':
    main()
