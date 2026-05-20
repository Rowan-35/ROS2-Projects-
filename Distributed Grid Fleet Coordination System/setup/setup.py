import rclpy
from rclpy.node import Node
import random
# Note: You will need to build your fleet_interfaces package first for this import to work
from fleet_interfaces.srv import RequestTask 

class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        # Provide the /request_task service [cite: 55, 56]
        self.srv = self.create_service(RequestTask, '/request_task', self.handle_task_request)
        
        # Generate at least 10 tasks [cite: 51] and store in a list [cite: 53]
        self.tasks = []
        for _ in range(10):
            task = {
                'pickup': (random.randint(0, 7), random.randint(0, 7)),
                'dropoff': (random.randint(0, 7), random.randint(0, 7))
            }
            self.tasks.append(task)
            
        self.completed_tasks = 0 # Keep count of completed tasks [cite: 59]
        self.get_logger().info('Task Manager initialized with 10 tasks.')

    def handle_task_request(self, request, response):
        # Assign one task at a time [cite: 57]
        if len(self.tasks) > 0:
            assigned_task = self.tasks.pop(0) # Never assign same task twice [cite: 58]
            response.success = True
            response.pickup_x = assigned_task['pickup'][0]
            response.pickup_y = assigned_task['pickup'][1]
            response.dropoff_x = assigned_task['dropoff'][0]
            response.dropoff_y = assigned_task['dropoff'][1]
            self.get_logger().info(f"Assigned task to {request.vehicle_id}")
        else:
            response.success = False
            self.get_logger().info("No more tasks available.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
