"""
this module is creating an environment model
"""
import math

import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node

from sf_msgs.msg import EMM, DetectedObject, ParkingSpot

class Helper(Node):
    """
    this node is subscribing to two topics, merging the information
    and publishing the data as a EMM-message
    """
    object_class_dict = {
            0 : 0, #unknown
            4 : 1, #pedestrian
            1 : 5, #passengerCar
            5 : 15 #roadSideUnit
        }

    def __init__(self):
        super().__init__("env_model")

        self.angle_min = -180
        self.angle_max = 180
        self.ranges = []
        self.counter = 10

        # subscriber
        self.create_subscription(EMM, "/env_model", self.emm_callback, 10)

        self.get_logger().info("helper")

    def emm_callback(self, emm_msg:EMM):
        if self.counter == 0:
            plt.hist(self.ranges, bins=300)
            plt.show()

        angles_with_ranges = list(zip(emm_msg.angles, emm_msg.ranges))
        for distance in angles_with_ranges:
            self.ranges.append(distance[1])
        self.counter -= 1

def main(args=None):
    """
    initializing and startign the node
    """
    rclpy.init(args=args)
    node = Helper()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
