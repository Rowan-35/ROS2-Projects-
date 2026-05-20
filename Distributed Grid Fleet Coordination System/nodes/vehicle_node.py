import sys
import rclpy
from rclpy.node import Node
from fleet_interfaces.msg import VehicleState
from fleet_interfaces.srv import RequestTask, RequestMove

class VehicleNode(Node):
    def __init__(self, vehicle_id):
        # Initialize the node with the specific vehicle ID (e.g., 'vehicle1')
        super().__init__(f'{vehicle_id}_node')
        self.vehicle_id = vehicle_id
        
        # 1. Setup Publishers and Service Clients
        self.state_pub = self.create_publisher(VehicleState, '/vehicle_state', 10) # [cite: 97]
        self.task_client = self.create_client(RequestTask, '/request_task') # [cite: 100]
        self.move_client = self.create_client(RequestMove, '/request_move') # [cite: 101]
        
        # 2. Internal State Machine Initialization 
        self.state = 'IDLE' # [cite: 42]
        self.previous_state = 'IDLE' # Remembers what to do after 'WAITING'
        
        # Logical grid coordinates (starting at 0,0) [cite: 13]
        self.current_x = 0  
        self.current_y = 0
        self.target_x = -1
        self.target_y = -1
        self.dropoff_x = -1
        self.dropoff_y = -1
        
        # 3. Timer for gradual movement delay (1 step per second) 
        self.timer = self.create_timer(1.0, self.state_machine_loop)
        self.get_logger().info(f'{self.vehicle_id} initialized and IDLE.')

    def publish_status(self):
        # Publish current position and state [cite: 31, 32, 34]
        msg = VehicleState()
        msg.vehicle_id = self.vehicle_id
        msg.current_state = self.state
        msg.x = self.current_x
        msg.y = self.current_y
        self.state_pub.publish(msg)

    def state_machine_loop(self):
        self.publish_status()
        
        if self.state == 'IDLE': # [cite: 42]
            self.state = 'REQUEST_TASK' # [cite: 43]
            
        elif self.state == 'REQUEST_TASK': # [cite: 43]
            if self.task_client.wait_for_service(timeout_sec=0.5):
                req = RequestTask.Request()
                req.vehicle_id = self.vehicle_id
                future = self.task_client.call_async(req) # Request task via service [cite: 39]
                future.add_done_callback(self.task_response_callback)
                self.state = 'WAITING' # [cite: 46]
            
        elif self.state == 'MOVING_TO_PICKUP': # [cite: 44]
            self.step_towards(self.target_x, self.target_y, 'MOVING_TO_DROPOFF') # [cite: 45]
            
        elif self.state == 'MOVING_TO_DROPOFF': # [cite: 45]
            self.step_towards(self.dropoff_x, self.dropoff_y, 'FINISHED') # [cite: 47]
            
        elif self.state == 'FINISHED': # [cite: 47]
            self.get_logger().info('Task Complete! Ready for new task.') # [cite: 24, 25]
            self.state = 'IDLE' # [cite: 42]

    def task_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                # Receive Pickup and Drop-off locations [cite: 18, 19, 20]
                self.target_x = response.pickup_x
                self.target_y = response.pickup_y
                self.dropoff_x = response.dropoff_x
                self.dropoff_y = response.dropoff_y
                self.get_logger().info(f'Task received! Pickup: ({self.target_x},{self.target_y})')
                self.state = 'MOVING_TO_PICKUP' # [cite: 44]
            else:
                self.state = 'IDLE' # Try again on the next loop
        except Exception as e:
            self.get_logger().error(f'Task request failed: {e}')
            self.state = 'IDLE'

    def step_towards(self, tx, ty, next_state):
        # Check if we have arrived
        if self.current_x == tx and self.current_y == ty:
            self.state = next_state
            return

        # Calculate next single-cell move (up, down, left, or right) [cite: 85, 87, 88, 89, 90]
        next_x, next_y = self.current_x, self.current_y
        if self.current_x < tx: 
            next_x += 1
        elif self.current_x > tx: 
            next_x -= 1
        elif self.current_y < ty: 
            next_y += 1
        elif self.current_y > ty: 
            next_y -= 1
            
        # Request permission before entering a cell 
        if self.move_client.wait_for_service(timeout_sec=0.5):
            req = RequestMove.Request()
            req.vehicle_id = self.vehicle_id
            req.current_x = self.current_x
            req.current_y = self.current_y
            req.target_x = next_x
            req.target_y = next_y
            
            future = self.move_client.call_async(req) # Request move permission via service [cite: 40]
            future.add_done_callback(lambda f: self.move_response_callback(f, next_x, next_y))
            
            self.previous_state = self.state # Remember what we were doing
            self.state = 'WAITING' # [cite: 46]

    def move_response_callback(self, future, next_x, next_y):
        try:
            response = future.result()
            if response.approved:
                self.current_x = next_x
                self.current_y = next_y
            # Resume previous activity whether approved (moved) or rejected (try again)
            self.state = self.previous_state 
        except Exception as e:
            self.get_logger().error(f'Move request failed: {e}')
            self.state = self.previous_state

def main(args=None):
    rclpy.init(args=args)
    
    # Grab the vehicle ID from the command line arguments 
    vehicle_id = 'vehicle1'
    if len(sys.argv) > 1:
        vehicle_id = sys.argv[1]
        
    node = VehicleNode(vehicle_id)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
