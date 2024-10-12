import rclpy
import unittest
from unittest.mock import MagicMock
from sf_v2x_com.receiver import CamReceiverNode
from v2x.msg import CAM
from sf_msgs.msg import VehiclesPosition

class TestV2XReceiver(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.receiver_class = CamReceiverNode()
        self.publisher = MagicMock()
        self.cam_msg = CAM()
        self.receiver_class.v_pos = VehiclesPosition()

    def tearDown(self):
        self.receiver_class.destroy_node()
        rclpy.shutdown()

    # TC_1: test if the cam callback is working for valid vehicle, valid vehicle id is 9

    def test_cam_callback_vehicle_9(self):

        self.receiver_class.lat_0 = 37.7749
        self.receiver_class.lon_0 = -122.4194
        self.receiver_class.h_0 = 0.0

        self.cam_msg.header.station_id = 9
        self.cam_msg.cam.cam_parameters.basic_container.reference_position.latitude = 377749000
        self.cam_msg.cam.cam_parameters.basic_container.reference_position.longitude = -1224194000
        self.cam_msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value = 1800

        self.receiver_class.cam_callback(self.cam_msg)

        #check if the x_coordinate is as expected
        self.assertAlmostEqual(self.receiver_class.v_pos.x_coordinate, 0, places=2)
        #check if the y_coordinate is as expected
        self.assertAlmostEqual(self.receiver_class.v_pos.y_coordinate, 0, places=2)
        #check if the yaw angle is as expected
        self.assertAlmostEqual(self.receiver_class.v_pos.yaw_angle, 180.0, places=2)

    # TC_2: test if the cam callback is working for invalid vehicle
    def test_cam_callback_invalid_vehicle(self):
        self.cam_msg.header.station_id = 8
        self.receiver_class.cam_callback(self.cam_msg)
        self.receiver_class.publisher.publish.assert_not_called()
        
if __name__ == '__main__':
    unittest.main()