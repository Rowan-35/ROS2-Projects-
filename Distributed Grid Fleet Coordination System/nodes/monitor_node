import rclpy
from rclpy.node import Node
import time
from fleet_interfaces.msg import VehicleState

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')
        
        # Subscribe to vehicle states 
        self.state_sub = self.create_subscription(
            VehicleState, 
            '/vehicle_state', 
            self.state_callback, 
            10)
        
        # Dictionaries to track what is happening
        self.vehicle_states = {}
        self.waiting_start_times = {}
        self.completed_tasks = 0
        
        # Create a timer to print the dashboard every 2 seconds
        self.timer = self.create_timer(2.0, self.print_dashboard)
        self.get_logger().info('Monitor Node initialized. Watching system...')

    def state_callback(self, msg):
        vid = msg.vehicle_id
        current_state = msg.current_state
        
        # Check if a task was just completed (transitioned to FINISHED)
        if vid in self.vehicle_states:
            if self.vehicle_states[vid] != 'FINISHED' and current_state == 'FINISHED':
                self.completed_tasks += 1
                
        # Update the tracked state
        self.vehicle_states[vid] = current_state
        
        # Track how long vehicles are in the WAITING state
        if current_state == 'WAITING':
            if vid not in self.waiting_start_times:
                self.waiting_start_times[vid] = time.time()
        else:
            # If they are no longer waiting, clear their wait timer
            if vid in self.waiting_start_times:
                del self.waiting_start_times[vid]

    def print_dashboard(self):
        active_tasks = []
        waiting_vehicles = []
        
        current_time = time.time()
        
        # Categorize the vehicles based on their states
        for vid, state in self.vehicle_states.items():
            if state in ['MOVING_TO_PICKUP', 'MOVING_TO_DROPOFF']:
                active_tasks.append(vid)
            elif state == 'WAITING':
                waiting_vehicles.append(vid)
                
                # Detect if a vehicle is stuck for more than 10 seconds 
                if vid in self.waiting_start_times:
                    wait_duration = current_time - self.waiting_start_times[vid]
                    if wait_duration > 10.0:
                        self.get_logger().error(f"STUCK VEHICLE ALERT: {vid} has been waiting for {wait_duration:.1f} seconds!")

        # Print the required dashboard information [cite: 74]
        print("\n" + "="*30)
        print("🚥 SYSTEM MONITOR DASHBOARD 🚥")
        print("="*30)
        # Print active tasks 
        print(f"Active Tasks:     {', '.join(active_tasks) if active_tasks else 'None'}")
        # Print waiting vehicles 
        print(f"Waiting Vehicles: {', '.join(waiting_vehicles) if waiting_vehicles else 'None'}")
        # Print completed tasks 
        print(f"Completed Tasks:  {self.completed_tasks}")
        print("="*30 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
