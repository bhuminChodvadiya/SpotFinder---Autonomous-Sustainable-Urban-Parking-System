import unittest
import rclpy
from std_msgs.msg import Bool
from sf_msgs.msg import EgoPosition
from sf_mi.sf_mi import MobileInterface

class TestMobileInterface(unittest.TestCase):
    """
    Unit tests for the MobileInterface node that integrates ROS2 with a Flask server to manage vehicle data.
    """

    def setUp(self):
        rclpy.init()
        self.node = MobileInterface()

    def tearDown(self):   
        self.node.destroy_node()
        rclpy.shutdown()

    # TC_MI_001: Test node initialization and logging.
    def test_init_node(self):
        """
        Test the initialization of the MobileInterface node to ensure all properties are set correctly and
        the node logs its running status.
        """
        self.assertIsNotNone(self.node)
        self.assertIsNone(self.node.ego_position)
        self.assertFalse(self.node.parking_position_reached)
        self.node.get_logger().info("Mobile interface is running")

    # TC_MI_002: Test the position update callback functionality.
    def test_position_callback(self):
        """
        Simulate receiving an EgoPosition message and verify that the node updates its internal state
        correctly.
        """
        ego_position_msg = EgoPosition()
        ego_position_msg.x_coordinate = 150.0  # Corrected to float
        ego_position_msg.y_coordinate = 250.0  # Corrected to float
        self.node.position_callback(ego_position_msg)

        self.assertEqual(self.node.ego_position.x_coordinate, 150.0)
        self.assertEqual(self.node.ego_position.y_coordinate, 250.0)

    # TC_MI_003: Test the parking status update callback functionality.
    def test_parking_callback(self):
        """
        Simulate receiving a Bool message indicating the parking position has been reached and verify
        that the node updates its internal state correctly.
        """
        parking_msg = Bool()
        parking_msg.data = True
        self.node.parking_callback(parking_msg)

        self.assertTrue(self.node.parking_position_reached)

    # TC_MI_004: Test the Flask endpoint for vehicle status retrieval.
    def test_get_vehicle_status_endpoint(self):
        """
        Test the Flask endpoint '/get_vehicle_status' to ensure it returns the correct vehicle status
        based on the current internal state of the node.
        """
        self.node.app.testing = True
        self.app = self.node.app.test_client()

        self.node.ego_position = EgoPosition()
        self.node.ego_position.x_coordinate = 150.0
        self.node.ego_position.y_coordinate = 250.0
        self.node.parking_position_reached = True

        response = self.app.get('/get_vehicle_status')
        self.assertEqual(response.status_code, 200)
        self.assertDictEqual(response.get_json(), {
            "x_coordinate": 150.0,
            "y_coordinate": 250.0,
            "parking_position_reached": True
        })

if __name__ == '__main__':
    unittest.main()

