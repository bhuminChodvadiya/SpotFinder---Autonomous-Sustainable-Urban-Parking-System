# Import libraries 
import rclpy
from rclpy.node import Node
from sf_msgs.msg import EgoPosition,SelectedParking 
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import math
import numpy as np

class ParkingManeuverNode(Node):
    def __init__(self):
        super().__init__("parking_maneuver") # Initialize the node

        # Timer for periodic checks
        self.timer_period = 0.01  # Set the timer period to 10 ms
        self.create_timer(self.timer_period, self.parking_maneuver) # Create a timer to call Node

        # Subscriber for ego position
        self.create_subscription(EgoPosition, '/ego_position', self.position_callback, 10)  # Subscribe to the /ego_position topic

        # Subscriber for selected parking point (coordinate )
        self.create_subscription(SelectedParking, '/final_parking_point', self.parking_spot_callback, 10) # Subscribe to the /final_parking_point topic

        # Publisher for Twist messages
        self.cmd_publisher = self.create_publisher(Twist, '/pm_cmd_vel', 10) # Create a publisher for Twist messages on the /pm_cmd_vel topic

         # Publisher for boolean message
        self.completion_publisher = self.create_publisher(Bool, '/parking_reached_msg', 10) # Create a publisher for Bool messages on the /parking_reached_msg topic
        self.create_timer(self.timer_period, self.publish_completion_message)

        # Initialize variables
        self.ego_position = None  # Initialize ego_position to None
        self.twist_msg = Twist() # Create an instance of Twist message
        self.parking_in_progress = False  # Flag to track if parking is in progress
        self.parking_spot = {
            "x_coordinate": None, # Initialize x_coordinate of parking spot
            "y_coordinate": None, # Initialize y_coordinate of parking spot
        }
        self.initial_position = None # Initialize initial_position to None
        self.moving_forward = True # Flag to indicate current movement direction (forward/backward)
        self.aligning_with_spot = True  # Flag to indicate if aligning with spot

    # Callback function to handle incoming ego position messages
    def position_callback(self, ego_position_msg):
        self.ego_position = ego_position_msg  # Update ego_position with incoming message
        # Print ego position information
        self.get_logger().info(f"Ego Position - x: {self.ego_position.x_coordinate:.2f}, y: {self.ego_position.y_coordinate:.2f}, yaw: {self.ego_position.yaw_angle:.2f}")
 
  # Callback function to handle incoming selected parking point messages
    def parking_spot_callback(self, selected_parking_msg):
        # Extract the last point from the selected parking message
        if selected_parking_msg.points:
            point = selected_parking_msg.points[-1] # Get the last point from the list
            self.parking_spot["x_coordinate"] = point.x # Set x_coordinate of parking spot
            self.parking_spot["y_coordinate"] = point.y + 0.75 # Set y_coordinate of parking spot with an offset
            self.get_logger().info(f"Received Parking Spot - x: {point.x:.2f}, y: {point.y:.2f}")
 
     # Function for parking maneuver 
    def parking_maneuver(self):
        if not self.parking_in_progress or self.ego_position is None or self.parking_spot["x_coordinate"] is None:
            return  # Exit if parking is not in progress, ego position is unavailable, or parking spot is not set

        if self.initial_position is None:
            self.initial_position = (self.ego_position.x_coordinate, self.ego_position.y_coordinate) # Set initial position if not already set

        self.get_logger().info("Parking maneuver is starting")

        if self.aligning_with_spot:
            # Calculate angle to align with parking spot
            steering_angle = self.calculate_parking_alignment() # Compute the required steering angle for alignment
            self.twist_msg.angular.z = steering_angle # Set the angular steering

            # Move forward to align with parking spot's y-coordinate
            if self.ego_position.y_coordinate < self.parking_spot["y_coordinate"] + 0.50:  # or self.ego_position.x_coordinate < self.parking_spot["x_coordinate"] - 0.75:
                self.twist_msg.linear.x = 0.3  # Move forward 
            else:
                self.twist_msg.linear.x = 0.0  # Stop once aligned
                self.aligning_with_spot = False  # Transition to backward movement

            self.publish_twist_message()

        else:  # Moving backward phase
            # Calculate angle to move backward into the parking spot
            steering_angle = self.calculate_parking_alignment() # Compute the required steering angle for backward movement


            # Check if the car is near the parking spot
            distance_to_spot = math.sqrt(
                (self.ego_position.x_coordinate - self.parking_spot["x_coordinate"]) ** 2 +
                (self.ego_position.y_coordinate - self.parking_spot["y_coordinate"]) ** 2
            ) # Compute the distance to the parking spot
            # If the car is very close to the parking spot, reduce the steering angle to straighten the wheels
            if distance_to_spot < 0.90:
                self.get_logger().info(f"Straightening wheels, distance to spot: {distance_to_spot:.2f}m")
                steering_angle = max(-5.0, min(5.0, steering_angle))  # Reduce steering angle for final alignment

            self.twist_msg.angular.z = steering_angle

            # Move backward slowly
            self.twist_msg.linear.x = -0.3

            self.publish_twist_message()

            # Check if the car has reached the parking spot
            if (self.ego_position.x_coordinate - self.parking_spot["x_coordinate"]) > 0.10:
                self.twist_msg.linear.x = 0.0  # Stop the car
                self.twist_msg.angular.z = 0.0  # Straighten wheels
                self.publish_twist_message()
                #self.get_logger().info("Parking maneuver completed")
                self.parking_in_progress = False
                self.publish_completion_message()  # Publish message after parking is done 

    def calculate_parking_alignment(self):
        if self.ego_position is None:
            return 0.0  # Return 0 steering angle if ego position is not available
        # Retrieve parking spot coordinates from self
        target_x = self.parking_spot["x_coordinate"]
        target_y = self.parking_spot["y_coordinate"]

        # Retrieve ego position coordinates from self
        car_x = self.ego_position.x_coordinate
        car_y = self.ego_position.y_coordinate

        # Print ego position and parking spot information
        self.get_logger().info(f"Parking Spot - x: {target_x:.2f}, y: {target_y:.2f}")
        self.get_logger().info(f"Ego Position - x: {car_x:.2f}, y: {car_y:.2f}, yaw: {self.ego_position.yaw_angle:.2f}")

        # Calculate angle to align with parking spot perpendicularly
        angle_to_target = math.atan2(target_y - car_y, target_x - car_x)  # Compute angle to the target
        perpendicular_angle = angle_to_target + math.pi / 2  # Perpendicular angle

        # Normalize steering angle to be within -pi to pi
        steering_angle = perpendicular_angle - self.ego_position.yaw_angle # Compute steering angle
        steering_angle = (steering_angle + math.pi) % (2 * math.pi) - math.pi # Normalize angle

        # Convert steering angle to degrees
        steering_angle_deg = np.degrees(steering_angle)  # Convert angle to degrees

        self.get_logger().info(f"Calculated Steering Angle before normalization: {steering_angle_deg:.2f} degrees")

        # Ensure the normalized angle fits within -30 to 30 degrees
        normalized_steering_angle_deg = max(-30.0, min(30.0, steering_angle_deg))

        self.get_logger().info(f"Normalized Steering Angle: {normalized_steering_angle_deg:.2f} degrees")

        return normalized_steering_angle_deg # Return the normalized steering angle
    
    def publish_twist_message(self):
        self.cmd_publisher.publish(self.twist_msg) # Publish the Twist message
        self.get_logger().info(f"Published Twist message: linear.x = {self.twist_msg.linear.x}, angular.z = {self.twist_msg.angular.z}")

    def publish_completion_message(self):
            completion_msg = Bool() # Create a Bool message
            completion_msg.data = True # Set message data to True
            self.completion_publisher.publish(completion_msg)  # Publish the completion message
            self.get_logger().info("Published completion message: Parking completed")
    def start_parking(self):
        self.parking_in_progress = True # Set flag to indicate parking is in progress
        self.initial_position = None  # Reset initial position
        self.moving_forward = True # Set movement direction to forward
        self.aligning_with_spot = True  # Reset alignment state when starting parking
        
def main(args=None):
    rclpy.init(args=args) 
    node = ParkingManeuverNode() # Create an instance of the ParkingManeuverNode
    node.start_parking() # Start the parking maneuver
    rclpy.spin(node) # Keep the node spinning to handle callbacks
    rclpy.shutdown()

if __name__ == '__main__':
    main()
