#!/usr/bin/env python3

# Refrence of emergency brake function and time to collision(TTC) calculation[https://github.com/f1tenth/f1tenth_lab2_template.git].
import rclpy
from rclpy.node import Node
from sf_msgs.msg import EgoPosition, EMM , SelectedParking
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

class DrivingController(Node):
    def __init__(self):
        super().__init__("driving_controller")

        # timer init
        timer_period = 0.01  # Timer period
        self.movement_timer = self.create_timer(timer_period, self.car_movment)  # Timer for car movement

        # subscriber
        self.create_subscription(EgoPosition, '/ego_position', self.position_callback, 10)  # Subscriber for ego position data
        self.create_subscription(Path, '/planned_path', self.path_callback, 10)  # Subscriber for path data
        self.create_subscription(SelectedParking, '/final_parking_point', self.final_parking_callback, 10)  # Subscriber for final parking point
        self.create_subscription(EMM, '/env_model', self.environment_model_callback, 10)  # Subscriber for environment model data
        self.create_subscription(Twist, '/pm_cmd_vel', self.parking_callback, 10)
        self.create_subscription(Twist, '/pp_cmd_vel', self.pure_pursuit_callback, 10)

        # publisher
        self.cmd_publisher = self.create_publisher(Twist, "/cmd_vel",10)  #Publisher for Twist messages

        # init variables
        self.ego_position = None   # Initialize variable for ego position
        self.path_data = None     # Initialize variable for route data
        self.car_max_velocity = 0.5  # Maximum velocity of the car
        self.car_velocity = 0.0    # Initialize current car velocity
        self.twist_msg = Twist()   # Twist message object
        self.twist_msg.angular.z = 5.5  # Angular velocity in the Twist message
        self.angles_with_ranges = None  # Initialize variable for angles with ranges
        self.timeout = None  # # Initialize timeout variable
        self.pp_twist = None
        self.pm_twist = None
        self.is_emergency = False
        self.final_parking_point = None
        self.in_parking = False

    def position_callback(self, ego_position_msg):
        '''
        Callback function for ego position data
        '''
        self.ego_position = ego_position_msg

    def path_callback(self, path_data_msg):
        '''
        Callback function for route data
        '''
        if path_data_msg is not None:
            self.path_data = path_data_msg

    def final_parking_callback(self, msg):
        self.final_parking_point = msg.points

    def environment_model_callback(self, env_model_msg):
        '''
        Callback function for environment model data
        '''
        self.angles_with_ranges = zip(env_model_msg.angles, env_model_msg.ranges)

    def parking_callback(self, msg):
        self.pm_twist = msg 

    def pure_pursuit_callback(self, msg):
        self.pp_twist = msg

    # different driving states
    def car_movment(self):
        '''
        Function for car movement
        '''
        if self.ego_position is None:  # Check if ego position data is available
            self.get_logger().info("Ego position not available")
            return
        
        if self.path_data is None or len(self.path_data.poses) == 0:  # Check if route data is available
            self.get_logger().info("Path not available")
            return
        
        if self.angles_with_ranges is None:  # Check if environment model data is available
            self.get_logger().info("Environment model not available")
            return
        
        if self.pp_twist is None:
            self.get_logger().info("Pure pursuit not available")
            return
        
        if self.pm_twist is None:
            self.get_logger().info("Parking manoeuver not available")
            return

        if self.emergency_brake():  # Check for emergency braking condition
            self.get_logger().info("Emergency brake!!!!!!!!!!")
            self.decelerate()
            return

        if self.reached_final_waypoint():  # Check if the final_waypoint is reached
            if not self.is_emergency:
                self.get_logger().info("Final waypoint reached")
                self.parking_manoeuver()
                self.publish_twist_message()
        else:
            if not self.is_emergency:            
                self.twist_msg = self.pp_twist
                if self.twist_msg is not None:
                    self.car_velocity = self.twist_msg.linear.x
                    self.get_logger().info("Pure pursuit is running")
                    self.publish_twist_message()

    def decelerate(self):
        '''
        Function for decelerating the car
        '''
        self.get_logger().info("decelerate")
        self.car_velocity = 0.0  # Car velocity to zero
        self.twist_msg.linear.x = float(self.car_velocity)   # Set linear velocity in the Twist message
        
    def reached_final_waypoint(self):
        '''
        Function to check if the final_waypoint is reached and take appropriate actions
        '''
        if (self.final_parking_point != None):
            final_parking_point_x = self.final_parking_point[0].x  # Get the final destination pose
            final_parking_point_y = self.final_parking_point[0].y 
            car_x = self.ego_position.x_coordinate  # x-coordinate of the car
            car_y = self.ego_position.y_coordinate  # y-coordinate of the car

            # Check if the car is within a certain range (0.3 units) of the final waypoint
            final_parking_xlim = final_parking_point_x - 1.25 <= car_x <= final_parking_point_x + 1.25
            final_parking_ylim = final_parking_point_y - 0.6 <= car_y <= final_parking_point_y + 0.75
            if (final_parking_xlim and final_parking_ylim) or (self.pp_twist.linear.x==0 and self.pp_twist.angular.z==0) or (self.in_parking):
                return True
    
    def parking_manoeuver(self):
        if self.pm_twist is not None:
            self.twist_msg = self.pm_twist  # Start parking maneuver
            self.car_velocity = self.twist_msg.linear.x
            self.get_logger().info("Parking manoever is running")
            self.in_parking = True

    # define car_velocity which is published from pp and pm_cmd_vel
    def emergency_brake(self):
        '''
        Function to check for emergency braking condition
        '''
        if self.car_velocity == 0.0 and self.pp_twist is not None:
            self.car_velocity = self.pp_twist.linear.x

       # vehicle driving forward
        distances = []  # Initialize list to store distances
        if self.car_velocity > 0:  # Check if car is moving forward
            distances = self.filter_angles(-100, -80, 0.15)  # Filter angles for forward movement
        elif self.car_velocity < 0:  # Check if car is moving backward
            distances = self.filter_angles(80, 100, 0.15) # Filter angles for backward movement

        # calculate TTC
        if distances:  # Check if distances are available
            min_distance = min(list(zip(*distances))[1])  # Calculate minimum distance
            ttc = min_distance/self.car_velocity  # Calculate time-to-collision
            if ttc <= 2:  # Check if time-to-collision is less than or equal to 2 seconds
                self.is_emergency = True
                return True  # Return True for emergency braking
            else:
                self.is_emergency = False
        return False  # Return False otherwise

    def filter_angles(self, angle_min, angle_max, min_range):
        '''
        Function to filter angles based on range
        '''
        return [tupel for tupel in self.angles_with_ranges if (angle_min <= tupel[0] <= angle_max and tupel[1] >= min_range)]   # Filter angles based on specified range

    def publish_twist_message(self):
        '''
        Function to publish Twist message
        '''
        if self.twist_msg is not None:
            self.cmd_publisher.publish(self.twist_msg)  # Publish the Twist message

# Main function to initialize the node, spin and shutdown
def main(args=None):
    rclpy.init(args=args)
    node = DrivingController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
