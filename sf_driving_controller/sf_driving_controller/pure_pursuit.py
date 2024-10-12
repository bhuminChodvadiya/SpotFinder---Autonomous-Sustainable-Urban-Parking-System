#!/usr/bin/env python3

# This pure pursuit controller is adpated majorly from noetic_pure_pursuit[https://github.com/hackerjeff705/noetic_pure_pursuit.git] by hackerjeff705.


import rclpy
from rclpy.node import Node
import math
import numpy as np
from numpy import linalg as la
import matplotlib.pyplot as plt
from matplotlib import patches
from sf_msgs.msg import EgoPosition
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist

class PurePursuitControllerNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_controller")
        
        # subscriber
        self.create_subscription(Twist, "/cmd_vel_feedback", self.pose_callback, 10)
        self.create_subscription(EgoPosition, "/ego_position", self.ego_position_callback, 10)
        self.create_subscription(Path, "/planned_path", self.path_callback, 10)

        # publisher
        self.control_publisher = self.create_publisher(Twist, "/pp_cmd_vel", 10)
        
        # timer for pure pursuit
        timer_period = 0.01 # seconds
        self.timer = self.create_timer(timer_period, self.pure_pursuit_callback)
        
        # GLOBAL VARIABLES 
        self.xc = 0.1
        self.yc = 0.1
        self.yaw = 0.0
        self.vel = 0.0
        self.idx = 0
        self.waypoints =  np.zeros((2,2))
        self.v_prev_error = 0.0
        self.freqs = 10

        # Waypoints for testing
        #self.waypoints = np.array([[0.5,0.5], [1.5,0.5], [2.5,0.5], [3.05,1.75], [3.05,2.5], [3.50,3.5], [3.50,4.5], [3.75,5.5], [3.75,6.5]])
        #self.waypoints = np.array([[0.5,0.5], [1.5,0.5], [2.25,0.5], [3.25,1.5], [3.5,2.5], [3.5,3.5], [3.5,4.5], [3.5,5.5], [3.5,6.5]]) #try this first

        # CAR VARIABLES
        # WB is the wheel base of the vehicle = 0.5 m
        self.LOOKAHEAD = 1.75 # 1.5
        self.WB = 0.5 #1.5

        # PROGRAM VARIABLES
        self.pure_pursuit_flag = True
        self.show_animation = False
        
        self.get_logger().info("Pure pursuit node is running...")
        
    def pose_callback(self, feedback_data):
        """
        Get current state and calculate velocity of the vehicle
        """
        self.vel = la.norm(np.array([feedback_data.linear.x, feedback_data.linear.y, feedback_data.linear.z]),2)
        
    def ego_position_callback(self, data):
        """
        Get current pose of the vehicle
        """
        self.xc = data.x_coordinate
        self.yc = data.y_coordinate
        self.yaw = data.yaw_angle

    def path_callback(self, path_data):
        #Create waypoints from the path data
        waypoint = []
        waypoints = []
        if path_data != None:
            nodes = path_data.poses
            for node in nodes:
               waypoint = []
               waypoint_x = node.pose.position.x/100   #Converting to metres
               waypoint_y = node.pose.position.y/100   #Converting to metres
               waypoint.append(waypoint_x)
               waypoint.append(waypoint_y)
               waypoints.append(waypoint)
        self.waypoints = np.array(waypoints)


    def find_distance(self, x1, y1):
        distance = math.sqrt((x1 - self.xc) ** 2 + (y1 - self.yc) ** 2)
        return distance

    def find_distance_index_based(self,idx):
        # To prevent index error
        if idx+1>len(self.waypoints):
            idx = len(self.waypoints)-1
        x1 = float(self.waypoints[idx][0])
        y1 = float(self.waypoints[idx][1])
        distance = math.sqrt((x1 - self.xc) ** 2 + (y1 - self.yc) ** 2)
        return distance

    def find_nearest_waypoint(self):
        """
        Get closest idx to the vehicle
        """
        curr_xy = np.array([self.xc, self.yc])
        waypoints_xy = self.waypoints[:, :2]
        nearest_idx = np.argmin(np.sum((curr_xy - waypoints_xy)**2, axis=1))
        return nearest_idx

    def idx_close_to_lookahead(self, idx):
        """
        Get closest index to lookahead that is greater than the lookahead
        """
        while self.find_distance_index_based(idx) < self.LOOKAHEAD:
            idx += 1 
            # To prevent index error
            if idx+1>len(self.waypoints):
                return len(self.waypoints)-1
        return idx-1

    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        """
        Plot arrow
        """
        if not isinstance(x, float):
            for ix, iy, iyaw in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw)
        else:
            plt.arrow(x, y, length * math.cos(self.yaw), length * math.sin(self.yaw), fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)
            patches.Rectangle((self.xc,self.yc), 0.35,0.2)

    def pure_pursuit_callback(self):

        # Initialize the message, subscriber and publisher
        msg = Twist()

        cx = self.waypoints[:, 0]; cy = self.waypoints[:, 1]

        try:
            if self.pure_pursuit_flag:
                nearest_idx = self.find_nearest_waypoint()
                idx_near_lookahead = self.idx_close_to_lookahead(nearest_idx)
                target_x = float(self.waypoints[idx_near_lookahead][0])
                target_y = float(self.waypoints[idx_near_lookahead][1])

                # Velocity PID controller
                kp = 1.0 # 1.0
                kd = 0
                ki = 0

                dt = 1.0 / self.freqs
                # Fixed velocity for the vehicle
                v_desired = 0.5
                v_error = v_desired - self.vel

                P_vel = kp * v_error
                I_vel = v_error * dt
                D_vel = kd * (v_error - self.v_prev_error) / dt

                velocity = P_vel + I_vel + D_vel
                self.v_prev_error = v_error
                print(f"NEAREST INDEX = {nearest_idx}, output = {velocity}, velocity desired = {v_desired}, current = {self.vel}")


                """
                    PURE PURSUIT CONTROLLER
                """

                # calculate alpha (angle between the goal point and the path point)
                x_delta = target_x - self.xc
                y_delta = target_y - self.yc
                alpha = np.arctan(y_delta / x_delta) - self.yaw

                # front of the vehicle is 0 degrees right +90 and left -90 hence we need to convert our alpha
                if alpha > np.pi / 2:
                    alpha -= np.pi
                if alpha < -np.pi / 2:
                    alpha += np.pi

                # Set the lookahead distance depending on the speed
                lookahead = self.find_distance(target_x, target_y)
                steering_angle = np.degrees(np.arctan((2 * self.WB * np.sin(alpha)) / lookahead))

                # Set max wheel turning angle
                if steering_angle > 45.0:
                    steering_angle = 45.0
                elif steering_angle < -45.0:
                    steering_angle = -45.0

                # Publish messages
                msg.linear.x = velocity
                msg.angular.z = steering_angle
                self.control_publisher.publish(msg)

                # Plot map progression
                if self.show_animation:
                    plt.cla()
                    # For stopping simulation with the esc key.
                    plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
                    self.plot_arrow(float(self.xc), float(self.yc), float(self.yaw))
                    plt.plot(cx, cy, "-r", label = "course")
                    plt.plot(self.xc, self.yc, "-b", label = "trajectory")
                    plt.plot(target_x, target_y, "xg", label = "target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.title("Pure Pursuit Control" + str(1))
                    plt.pause(0.001)

        except IndexError:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.control_publisher.publish(msg)
            print("PURE PURSUIT COMPLETE --> COMPLETED ALL WAYPOINTS")


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
