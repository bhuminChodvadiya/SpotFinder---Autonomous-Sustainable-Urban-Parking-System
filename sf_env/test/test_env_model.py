import unittest
import rclpy
import math
import pytest

from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import LaserScan, Image

from sf_env.sf_env import EnvModel

class TestEnvModel(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.env_model = EnvModel()

    def tearDown(self):
        self.env_model.destroy_node()
        rclpy.shutdown() 

    # TC_01: test if the Node can being created
    def test_init_node(self):

        assert self.env_model.get_logger().info("env model is running")

    # TC_02: test if the Scan_callback is working properly
    def test_scan_callback(self):
        assert len(self.env_model.emm.ranges) == 0
        assert self.env_model.emm.angle_min == 0
        assert self.env_model.emm.angle_max == 0
        assert self.env_model.emm.angle_increment == 0

        laserscan_msg = LaserScan()
        laserscan_msg.ranges = [0.1, 1.1, 2.0, 4.0, 0.6]
        laserscan_msg.angle_min = -math.pi
        laserscan_msg.angle_max = math.pi
        laserscan_msg.angle_increment = 0.01

        self.env_model.scan_callback(laserscan_msg)

        assert [round(range, 1) for range in self.env_model.emm.ranges] == [0.1, 1.1, 2.0, 4.0, 0.6]
        assert self.env_model.emm.angle_min == -math.pi
        assert self.env_model.emm.angle_max == math.pi
        assert self.env_model.emm.angle_increment == 0.01


    # TC_03: 
    def test_detectnet_callback(self):

        detection_2d_msg = Detection2DArray()

        detections = []

        # create six detection with class ids from 1 to 6
        for i in range(1,7):
            detection = Detection2D()

            results = []

            result = ObjectHypothesisWithPose()
            result.id = rf'\x0{i}'
            result.score = 80.0+i
            results.append(result)

            detection.results = results
            detections.append(detection)

        detection_2d_msg.detections = detections
        print(f"--------------------------{ord(detection_2d_msg.detections[1].results[0].id)}")
        
        self.env_model.detectnet_callback(detection_2d_msg)
        
        # check if parsing of the values was correct
        # there should be one parking spot detected
        print(f"--------------------------{len(self.env_model.emm.parking_spot_container)}")
        assert len(self.env_model.emm.parking_spot_container) == 1

        # there should be 3 objects detected
        assert len(self.env_model.emm.detected_objects_container) == 3

        # check values of the 1. detected Object
        assert self.env_model.emm.detected_objects_container[0].object_id == 0
        assert self.env_model.emm.detected_objects_container[0].object_class == 5
        assert self.env_model.emm.detected_objects_container[0].confidence == 81

        # check values of the 2. detected Object
        assert self.env_model.emm.detected_objects_container[0].object_id == 1
        assert self.env_model.emm.detected_objects_container[0].object_class == 1
        assert self.env_model.emm.detected_objects_container[0].confidence == 84

        # check values of the 3. detected Object
        assert self.env_model.emm.detected_objects_container[0].object_id == 2
        assert self.env_model.emm.detected_objects_container[0].object_class == 15
        assert self.env_model.emm.detected_objects_container[0].confidence == 85

    # TC_4: check if  depth_metadat_callback is working
    # this function is currently not in use and doesent provide new functionality
    def test_depth_metadata_callback(self):

        depth_metadata_msg = Image()
        data = [1,2,0,1]
        depth_metadata_msg.data = data

        self.env_model.depth_metadata_callback(depth_metadata_msg)


if __name__ == '__main__':
    unittest.main()