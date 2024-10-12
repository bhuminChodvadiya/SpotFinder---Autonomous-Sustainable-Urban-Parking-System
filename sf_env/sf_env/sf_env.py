"""
this module is creating an environment model
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import LaserScan
from sf_msgs.msg import EMM, DetectedObject, ParkingSpot

class EnvModel(Node):
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

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)

        # subscriber
        self.create_subscription(
            Detection2DArray, "/detectnet/detections", self.detectnet_callback, 10)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile)

        # publisher
        self.env_model_publisher = self.create_publisher(EMM, "/env_model", 10)

        # timer
        # in seconds equal to 33.33 hz
        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # init variables
        # empty message which will be published by env_model_publisher
        self.emm = EMM()

        self.get_logger().info("env model is running")

    def detectnet_callback(self, detection_2d_msg):
        """
        convert data from the Detection2DArray message
        """
        detected_objects_container = []
        parking_spot_container = []

        # variable to set the object id
        object_id = 0
        parking_spot_id = 0
        # for each detected object
        for detection in detection_2d_msg.detections:
            # extract class of detected object and check if the object is needed
            # if detection is a parkign spot
            print(detection.results[0].id)
            if ord(detection.results[0].id) == 2:
                parking_spot = ParkingSpot()

                parking_spot.id = parking_spot_id
                parking_spot_id += 1

                parking_spot_container.append(parking_spot)
            else:
                object_class_id = self.object_class_dict.get(ord(detection.results[0].id))

                if object_class_id is not None:
                    # create DetectedObject and insert the corresponding data from the detection
                    detected_object = DetectedObject()

                    detected_object.object_id = object_id
                    object_id += 1

                    detected_object.confidence = int(detection.results[0].score*100)

                    detected_object.object_class = object_class_id

                    detected_objects_container.append(detected_object)

        # updated detected Objects in the emm-message
        self.emm.parking_spot_container = parking_spot_container
        self.emm.detected_objects_container = detected_objects_container

    def scan_callback(self, laserscan_msg:LaserScan):
        """
        convert data from LaserScan message and insert it into the emm message
        """
        # Initialize list for angles with ranges
        angles = []
        # Set the starting angle offset in radians
        angle_start = -5.74
        angle_increment = laserscan_msg.angle_increment
        for i in range(0, len(laserscan_msg.ranges)):
            # Check if angle is within the range of -pi to pi
            if angle_start+angle_increment*i < math.pi:
                angles.append(math.degrees(angle_start + angle_increment * i))
            else:
                angles.append(math.degrees(angle_start - 2*math.pi + angle_increment * i))

        self.emm.ranges = laserscan_msg.ranges
        self.emm.angles = angles

    def timer_callback(self):
        """
        publish the emm_message peridical
        """
        self.env_model_publisher.publish(self.emm)

def main(args=None):
    """
    initializing and startign the node
    """
    rclpy.init(args=args)
    node = EnvModel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
