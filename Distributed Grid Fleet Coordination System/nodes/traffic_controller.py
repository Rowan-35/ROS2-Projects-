import rclpy
from rclpy.node import Node
import time
from fleet_interfaces.msg import VehicleState
from fleet_interfaces.srv import RequestMove

class TrafficController(Node):
    def __init__(self):
        super().__init__('traffic_controller')
        
        # Keep track of occupied cells and waiting times
        self.vehicle_positions = {}  # Format: {vehicle_id: (x, y)}
        self.waiting_vehicles = {}   # Format: {vehicle_id: wait_start_time}
        
        # 1. Subscribe to vehicle positions to know where everyone is
        self.state_sub = self.create_subscription(
            VehicleState, 
            '/vehicle_state', 
            self.state_callback, 
            10)
        
        # 2. Provide the /request_move service
        self.move_srv = self.create_service(
            RequestMove, 
            '/request_move', 
            self.handle_move_request)
            
        self.get_logger().info('Traffic Controller is running and watching the grid...')

    def state_callback(self, msg):
        # Update the known position of each vehicle based on their published state
        self.vehicle_positions[msg.vehicle_id] = (msg.x, msg.y)
        
        # If a vehicle successfully moved and is no longer waiting, stop tracking its wait time
        if msg.current_state != 'WAITING' and msg.vehicle_id in self.waiting_vehicles:
            del self.waiting_vehicles[msg.vehicle_id]

    def handle_move_request(self, request, response):
        target = (request.target_x, request.target_y)
        current = (request.current_x, request.current_y)
        vid = request.vehicle_id
        
        # Start the clock for this vehicle if it isn't already waiting
        if vid not in self.waiting_vehicles:
            self.waiting_vehicles[vid] = time.time()
            
        wait_time = time.time() - self.waiting_vehicles[vid]
        
        # Rule 1: Grid boundary check (Grid size: 8x8)
        if not (0 <= target[0] <= 7 and 0 <= target[1] <= 7):
            response.approved = False
            return response
            
        # Rule 2: Collision & Swap Prevention
        is_occupied = False
        occupant_id = None
        
        for other_id, pos in self.vehicle_positions.items():
            if other_id != vid and pos == target:
                is_occupied = True
                occupant_id = other_id
                break
                
        if is_occupied:
            response.approved = False
            self.get_logger().info(f"Move Rejected: {vid} wants {target}, but {occupant_id} is there.")
            
            # Rule 3: Detect deadlocks if waiting too long
            if wait_time > 5.0:
                self.get_logger().warn(f"DEADLOCK DETECTED! {vid} stuck for {wait_time:.1f}s waiting on {occupant_id}.")
                # Note: In a fully advanced system, giving 'priority' here would force the other vehicle 
                # to calculate a new sidestep route. To prevent a fatal crash, we maintain the rejection.
        else:
            # Rule 4: Handle multiple vehicles wanting the SAME empty cell (Priority)
            # If the cell is empty, the traffic controller approves it.
            # If two requested it at the same millisecond, ROS 2 processes them in order, 
            # naturally giving priority to the first request in the queue.
            response.approved = True
            self.get_logger().info(f"Move Approved: {vid} cleared to enter {target}")
            
            # Immediately reserve the cell so the next vehicle in line gets rejected
            self.vehicle_positions[vid] = target 
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrafficController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
