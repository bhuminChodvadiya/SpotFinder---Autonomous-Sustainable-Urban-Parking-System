import rclpy
import unittest
from sf_model_city_map.sf_viz_cars import VisualizationNode
from sf_msgs.msg import VehiclesPosition
import tf_transformations as tf
import pytest

@pytest.mark.rostest
class Testmcmap(unittest.TestCase):
    
    def setUp(self):
        rclpy.init()
        self.map_class = VisualizationNode()

    def tearDown(self):
        self.map_class.destroy_node()
        rclpy.shutdown()
        
    def test_vehicle_id_9(self):
        
        msg = VehiclesPosition()
        msg.vehicle_id = 9
        msg.x_coordinate = 1.0
        msg.y_coordinate = 2.0
        msg.yaw_angle = 0.0
        self.map_class.cam_position(msg)
        assert len(self.map_class.marker_array.markers) > 0
        
    def test_vehicle_id_10(self):
        msg = VehiclesPosition()
        msg.vehicle_id = 10
        msg.x_coordinate = 3.0
        msg.y_coordinate = 4.0
        msg.yaw_angle = 0.0
        self.map_class.cam_position(msg)
        #time.sleep(1)  # Wait for the marker to be published
        # Assert that the marker with ID 10 is published
        assert len(self.map_class.marker_array.markers) > 0
        
    def test_invalid_vehicle_id(self):
        msg = VehiclesPosition()
        msg.vehicle_id = 100  # An unsupported vehicle ID
        msg.x_coordinate = 5.0
        msg.y_coordinate = 6.0
        msg.yaw_angle = 0.0
        self.map_class.cam_position(msg)
        #time.sleep(1)  # Wait for the marker to be published
        # Assert that no markers are published
        assert len(self.map_class.marker_array.markers) == 0

if __name__ == '__main__':
    unittest.main()