import unittest
import rclpy
from rclpy.node import Node
from sf_v2x_com.transmitter import Tranmitter
from sf_msgs.msg import EgoPosition
from vision_msgs.msg import Detection2DArray

class TestTransmitter(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.transmitter = Tranmitter()

    def tearDown(self):
        self.transmitter.destroy_node()
        rclpy.shutdown()

    def test_ego_position_callback(self):
        self.ego_position_msg = EgoPosition()
        self.ego_position_msg.longitude = 10
        self.ego_position_msg.latitude = 20
        self.ego_position_msg.yaw_angle = 30.0
        self.ego_position_msg.velocity = 10.0
        self.ego_position_msg.acceleration = 2.0

        self.transmitter.ego_position_callback(self.ego_position_msg)

        # Check if CAM message was published
        self.assertTrue(len(self.transmitter.cam_publisher.get_message()) > 0)

        # Check if CAM message contains correct data
        cam_msg = self.transmitter.cam_publisher.get_message()[0]
        self.assertEqual(cam_msg.cam.cam_parameters.basic_container.reference_position.longitude, 10)
        self.assertEqual(cam_msg.cam.cam_parameters.basic_container.reference_position.latitude, 20)
        self.assertEqual(cam_msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.speed.speed_value, 1000)
        self.assertEqual(cam_msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.longitudinal_acceleration.longitudinal_acceleration_value, 200)
        self.assertEqual(cam_msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value, 300)

    def test_detectnet_callback_with_no_detections(self):
        detection_msg = Detection2DArray()
        
        self.lon = self.ego_position_msg.longitude
        self.lat = self.ego_position_msg.latitude
        self.transmitter.detectnet_callback(detection_msg)

        # Check if CPM message was published
        self.assertTrue(len(self.transmitter.cpm_publisher.get_message()) > 0)

        # Check if CPM message contains correct data
        cpm_msg = self.transmitter.cpm_publisher.get_message()[0]
        self.assertEqual(cpm_msg.payload.management_container.segmentation_info.total_msg_no, 1)
        self.assertEqual(cpm_msg.payload.management_container.segmentation_info.this_msg_no, 1)
        self.assertEqual(cpm_msg.payload.management_container.reference_position.longitude, 10)
        self.assertEqual(cpm_msg.payload.management_container.reference_position.latitude, 20)
        self.assertFalse(cpm_msg.payload.cpm_containers.originating_rsu_container_is_present)
        self.assertEqual(cpm_msg.payload.cpm_containers.sensor_information_container.sensor_information_container.type, 3)
        self.assertEqual(cpm_msg.payload.cpm_containers.perceived_object_container.number_of_perceived_objects, 0)

    def test_detectnet_callback_with_detections(self):
        detection_msg = Detection2DArray()
        # Simulating two detections
        detection_msg.detections = [
            {
                'results': [{'id': 'person', 'score': 0.9}]  # Assuming 'person' detection with high confidence
            },
            {
                'results': [{'id': 'car', 'score': 0.8}]  # Assuming 'car' detection with relatively high confidence
            }
        ]

        self.transmitter.detectnet_callback(detection_msg)

        # Check if CPM message was published
        self.assertTrue(len(self.transmitter.cpm_publisher.get_message()) > 0)

        # Check if CPM message contains correct data
        cpm_msg = self.transmitter.cpm_publisher.get_message()[0]
        self.assertEqual(cpm_msg.payload.management_container.segmentation_info.total_msg_no, 1)
        self.assertEqual(cpm_msg.payload.management_container.segmentation_info.this_msg_no, 1)
        self.assertEqual(cpm_msg.payload.management_container.reference_position.longitude, 10)
        self.assertEqual(cpm_msg.payload.management_container.reference_position.latitude, 20)
        self.assertFalse(cpm_msg.payload.cpm_containers.originating_rsu_container_is_present)
        self.assertEqual(cpm_msg.payload.cpm_containers.sensor_information_container.sensor_information_container.type, 3)
        self.assertEqual(cpm_msg.payload.cpm_containers.perceived_object_container.number_of_perceived_objects, 2)

        # Check if each perceived object has the correct classification
        self.assertEqual(len(cpm_msg.payload.cpm_containers.perceived_object_container.perceived_objects), 2)
        for perceived_object in cpm_msg.payload.cpm_containers.perceived_object_container.perceived_objects:
            for classification in perceived_object.classification:
                self.assertIn(classification.object_class.vehicle_sub_class, [0, 1, 5, 15])  # Checking if classification is valid
                self.assertGreaterEqual(classification.confidence, 0)  # Confidence should be non-negative
                self.assertLessEqual(classification.confidence, 100)  # Confidence should not exceed 100

if __name__ == '__main__':
    unittest.main()
